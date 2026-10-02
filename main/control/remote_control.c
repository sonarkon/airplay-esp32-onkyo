#include "remote_control.h"

#include "driver/gpio.h"
#include "esp_rom_gpio.h"
#include "esp_rom_sys.h"
#include "soc/gpio_struct.h"
#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "rtsp/rtsp_events.h"
#include <stdlib.h>
#include <string.h>

#define TAG "remote"

/* 1 tick = 1 us. Both protocols are specified in microseconds, and the
 * longest single interval (the 20 ms RI inter-frame gap) still fits the
 * 15-bit duration field. */
#define RMT_RESOLUTION_HZ 1000000
/* 48 is the default per-channel symbol memory on ESP32-S3. Our longest
 * frame (NEC, 34 symbols) fits comfortably, no need to span into a
 * neighbouring channel's block. */
#define RMT_MEM_SYMBOLS   48

/* NEC, as the Onkyo remote sends it */
#define NEC_HDR_MARK  9000
#define NEC_HDR_SPACE 4500
#define NEC_BIT_MARK  560
#define NEC_ONE_SPACE 1690
#define NEC_ZERO_SPACE 560
#define NEC_TAIL_SPACE 10000

/* RI: 12 bits, MSB first, no carrier */
#define RI_HDR_MARK  3000
#define RI_HDR_SPACE 1000
#define RI_BIT_MARK  1000
#define RI_ONE_SPACE 2000
#define RI_ZERO_SPACE 1000
#define RI_TAIL_SPACE 20000

#define IR_CARRIER_HZ 38000
#define IR_CARRIER_DUTY 0.33f

/* Every code here was sent at the receiver and the reaction confirmed on its
 * display — the labels are not inferred from the order they were captured in. */
static const remote_command_t s_commands[] = {
    {"opt",     true,  0x03D2, 0xA857, "Input OPT (TV, optical)"},
    {"volup",   true,  0x03D2, 0xFD02, "Volume up"},
    {"voldown", true,  0x03D2, 0xFC03, "Volume down"},
    {"mute",    true,  0x03D2, 0xFA05, "Mute"},
    {"line1",   true,  0x03D2, 0xF906, "Input LINE 1 (turntable)"},
    {"line2",   true,  0x03D2, 0xEE11, "Input LINE 2 (spare)"},
    {"line3",   true,  0x04D2, 0xB44B, "Input LINE 3 (AirPlay)"},
    {"cd",      true,  0x03D2, 0xA956, "Input CD/COAX"},
    {"tuner",   true,  0x03D2, 0xF40B, "Input TUNER"},
    {"phono",   true,  0x03D2, 0xF50A, "Input PHONO"},
    {"ipod",    true,  0x05D2, 0x619E, "Input iPod/USB"},
    {"dimmer",  true,  0x03D2, 0x6A95, "Display dimmer (steps)"},
    {"ir-power", true, 0x04D2, 0x34CB, "Power toggle over IR - prefer ri-on/ri-off"},
    {"ri-on",   false, 0,      0x020,  "Power on (discrete, selects CD/COAX)"},
    {"ri-off",  false, 0,      0x420,  "Standby (discrete)"},
};

#if CONFIG_REMOTE_AUTO_INPUT
static void auto_input_start(void);   /* Definition weiter unten */
#endif
#if CONFIG_REMOTE_POWER_SENSE
static esp_err_t power_sense_start(int ri_gpio); /* Definition weiter unten */
#endif

static rmt_channel_handle_t s_ir_chan = NULL;
static rmt_channel_handle_t s_ri_chan = NULL;
static rmt_encoder_handle_t s_encoder = NULL;
static SemaphoreHandle_t    s_lock    = NULL;

#if CONFIG_REMOTE_POWER_SENSE
static int                 s_ri_gpio       = -1;
static adc_oneshot_unit_handle_t s_adc     = NULL;
static adc_cali_handle_t   s_adc_cali_12   = NULL; /* ADC_ATTEN_DB_12 */
static adc_cali_handle_t   s_adc_cali_0    = NULL; /* ADC_ATTEN_DB_0: finest at the bottom */
static adc_channel_t       s_adc_channel   = 0;
static portMUX_TYPE        s_sense_mux     = portMUX_INITIALIZER_UNLOCKED;
/* The pad exactly as rmt_new_tx_channel() left it, captured before the ADC
 * driver touches it. gpio_config_as_analog() (called by
 * adc_oneshot_config_channel) switches the output off, so a snapshot taken
 * after that would preserve a dead output. */
/* True when CONFIG_REMOTE_POWER_SENSE_GPIO names a dedicated ADC pin wired to the
 * RI tip: the RI pad is then never touched, the ADC just reads the tip while
 * the transmitter keeps holding its own pad low. */
static bool                s_sense_ext     = false;
static uint32_t            s_pad_out_sel   = 0;
static bool                s_pad_was_out   = false;
static remote_power_sense_t s_sense        = {0};
#endif

const remote_command_t *remote_control_commands(size_t *count) {
  if (count) {
    *count = sizeof(s_commands) / sizeof(s_commands[0]);
  }
  return s_commands;
}

bool remote_control_available(void) {
  return s_ir_chan != NULL || s_ri_chan != NULL;
}

static esp_err_t make_channel(int gpio, bool with_carrier,
                              rmt_channel_handle_t *out) {
  rmt_tx_channel_config_t cfg = {
      .clk_src           = RMT_CLK_SRC_DEFAULT,
      .gpio_num          = gpio,
      .mem_block_symbols = RMT_MEM_SYMBOLS,
      .resolution_hz     = RMT_RESOLUTION_HZ,
      .trans_queue_depth = 4,
  };
  ESP_RETURN_ON_ERROR(rmt_new_tx_channel(&cfg, out), TAG,
                      "rmt_new_tx_channel(GPIO%d)", gpio);

  if (with_carrier) {
    /* The receiver's demodulator is tuned to 38 kHz and rejects everything
     * else — which is also why daylight and fluorescent lamps do not disturb
     * it. Value taken from the Pronto header of our own capture, not assumed. */
    rmt_carrier_config_t carrier = {
        .duty_cycle   = IR_CARRIER_DUTY,
        .frequency_hz = IR_CARRIER_HZ,
        .flags        = {.polarity_active_low = false},
    };
    ESP_RETURN_ON_ERROR(rmt_apply_carrier(*out, &carrier), TAG,
                        "rmt_apply_carrier");
  } else {
    /* RI is an unmodulated DC pulse train. Passing NULL removes any carrier. */
    ESP_RETURN_ON_ERROR(rmt_apply_carrier(*out, NULL), TAG, "clear carrier");
  }

  ESP_RETURN_ON_ERROR(rmt_enable(*out), TAG, "rmt_enable");
  return ESP_OK;
}

esp_err_t remote_control_init(void) {
  const int ir_gpio = CONFIG_REMOTE_IR_TX_GPIO;
  const int ri_gpio = CONFIG_REMOTE_RI_TX_GPIO;

  if (ir_gpio < 0 && ri_gpio < 0) {
    ESP_LOGI(TAG, "no transmitter configured, remote control disabled");
    return ESP_OK;
  }

  s_lock = xSemaphoreCreateMutex();
  ESP_RETURN_ON_FALSE(s_lock, ESP_ERR_NO_MEM, TAG, "mutex");

  rmt_copy_encoder_config_t enc_cfg = {};
  ESP_RETURN_ON_ERROR(rmt_new_copy_encoder(&enc_cfg, &s_encoder), TAG,
                      "rmt_new_copy_encoder");

  if (ir_gpio >= 0) {
    ESP_RETURN_ON_ERROR(make_channel(ir_gpio, true, &s_ir_chan), TAG, "IR");
    ESP_LOGI(TAG, "IR transmitter on GPIO%d, %d Hz carrier", ir_gpio,
             IR_CARRIER_HZ);
  }
  if (ri_gpio >= 0) {
    ESP_RETURN_ON_ERROR(make_channel(ri_gpio, false, &s_ri_chan), TAG, "RI");
    ESP_LOGI(TAG, "RI transmitter on GPIO%d", ri_gpio);
  }

#if CONFIG_REMOTE_POWER_SENSE
  if (ri_gpio >= 0) {
    esp_err_t err = power_sense_start(ri_gpio);
    if (err != ESP_OK) {
      ESP_LOGW(TAG, "power sense disabled: %s", esp_err_to_name(err));
    }
  }
#endif

#if CONFIG_REMOTE_AUTO_INPUT
  auto_input_start();
#endif
  return ESP_OK;
}

static esp_err_t transmit(rmt_channel_handle_t chan,
                          const rmt_symbol_word_t *symbols, size_t count) {
  rmt_transmit_config_t tx_cfg = {.loop_count = 0};
  ESP_RETURN_ON_ERROR(
      rmt_transmit(chan, s_encoder, symbols, count * sizeof(symbols[0]),
                   &tx_cfg),
      TAG, "rmt_transmit");
  /* rmt_tx_wait_all_done() has been confirmed, by watching the receiver
   * react correctly to every command across repeated real-world tests, to
   * sometimes report ESP_ERR_TIMEOUT even though the frame was sent
   * correctly — a completion-detection bug in this ESP-IDF/RMT driver
   * combination, not a sign the command was lost. A retry here does not
   * help (confirmed): the transmission already happened, and a second
   * rmt_transmit() call right after a misreported timeout reliably fails
   * for real, for reasons not pinned down either. So: send once, report
   * whatever rmt_tx_wait_all_done() says, and do not treat a false
   * ESP_ERR_TIMEOUT here as proof the command never reached the line —
   * see docs/HARDWARE.md for how this was established. */
  return rmt_tx_wait_all_done(chan, pdMS_TO_TICKS(500));
}

esp_err_t remote_control_send_ir(uint16_t address, uint16_t command) {
  ESP_RETURN_ON_FALSE(s_ir_chan, ESP_ERR_INVALID_STATE, TAG,
                      "IR transmitter not configured");

  /* Address and command each go out as 16 bits, least significant bit first.
   * That is the wire order NEC uses, and writing it this way means the values
   * from the code book can be used verbatim. */
  const uint32_t frame = ((uint32_t)command << 16) | address;

  rmt_symbol_word_t sym[34];
  size_t n = 0;
  sym[n++] = (rmt_symbol_word_t){.level0    = 1,
                                 .duration0 = NEC_HDR_MARK,
                                 .level1    = 0,
                                 .duration1 = NEC_HDR_SPACE};
  for (int i = 0; i < 32; i++) {
    sym[n++] = (rmt_symbol_word_t){
        .level0    = 1,
        .duration0 = NEC_BIT_MARK,
        .level1    = 0,
        .duration1 = (frame >> i) & 1 ? NEC_ONE_SPACE : NEC_ZERO_SPACE};
  }
  sym[n++] = (rmt_symbol_word_t){.level0    = 1,
                                 .duration0 = NEC_BIT_MARK,
                                 .level1    = 0,
                                 .duration1 = NEC_TAIL_SPACE};
  return transmit(s_ir_chan, sym, n);
}

esp_err_t remote_control_send_ri(uint16_t code) {
  ESP_RETURN_ON_FALSE(s_ri_chan, ESP_ERR_INVALID_STATE, TAG,
                      "RI transmitter not configured");
  ESP_RETURN_ON_FALSE(code <= 0xFFF, ESP_ERR_INVALID_ARG, TAG,
                      "RI codes are 12 bits");

  rmt_symbol_word_t sym[14];
  size_t n = 0;
  sym[n++] = (rmt_symbol_word_t){.level0    = 1,
                                 .duration0 = RI_HDR_MARK,
                                 .level1    = 0,
                                 .duration1 = RI_HDR_SPACE};
  for (int i = 11; i >= 0; i--) { /* MSB first, unlike NEC */
    sym[n++] = (rmt_symbol_word_t){
        .level0    = 1,
        .duration0 = RI_BIT_MARK,
        .level1    = 0,
        .duration1 = (code >> i) & 1 ? RI_ONE_SPACE : RI_ZERO_SPACE};
  }
  sym[n++] = (rmt_symbol_word_t){.level0    = 1,
                                 .duration0 = RI_BIT_MARK,
                                 .level1    = 0,
                                 .duration1 = RI_TAIL_SPACE};
  return transmit(s_ri_chan, sym, n);
}

/* ── Automatik: Receiver einschalten, wenn AirPlay startet ───────────────
 *
 * Die Firmware weiß selbst, wann ein Stream beginnt — sie muss dafür weder
 * Home Assistant fragen noch von dort gesteuert werden. Das ist der
 * schnellere und der robustere Weg: kein Netzwerkweg, keine Abhängigkeit
 * von einem laufenden Automationsserver.
 *
 * Bewusst nur einschalten, nie ausschalten. Das Ausschalten hängt an der
 * Automation, die dem Fernseher folgt; würden beide Seiten den Zustand
 * setzen, kämen sie einander in die Quere.
 *
 * Liegt das Audio auf CD/COAX, genügt der RI-Befehl 0x20 allein: er wählt
 * genau diesen Eingang und weckt das Gerät dabei. Der zweite Schritt ist
 * dann leer zu lassen (REMOTE_AUTO_INPUT_CMD = ""), und die Sequenz ist
 * nach 60 ms erledigt statt nach drei Sekunden.
 */
#if CONFIG_REMOTE_AUTO_INPUT

static TaskHandle_t s_auto_task = NULL;

static void auto_input_task(void *arg) {
  (void)arg;
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    ESP_LOGI(TAG, "playback started, waking the receiver");
    esp_err_t err = remote_control_send("ri-on", 1);
    if (err != ESP_OK) {
      ESP_LOGW(TAG, "wake failed: %s", esp_err_to_name(err));
    }

    /* Ein leerer Eingangsbefehl heißt: das Wecken hat den Eingang schon
     * mitgewählt, hier ist nichts mehr zu tun. */
    if (CONFIG_REMOTE_AUTO_INPUT_CMD[0] != '\0') {
      /* Nach dem Aufwecken braucht das Gerät einen Moment, bevor es einen
       * Eingangsbefehl über IR annimmt. */
      if (CONFIG_REMOTE_AUTO_INPUT_DELAY_MS > 0) {
        vTaskDelay(pdMS_TO_TICKS(CONFIG_REMOTE_AUTO_INPUT_DELAY_MS));
      }
      err = remote_control_send(CONFIG_REMOTE_AUTO_INPUT_CMD, 1);
      if (err != ESP_OK) {
        ESP_LOGW(TAG, "input select '%s' failed: %s",
                 CONFIG_REMOTE_AUTO_INPUT_CMD, esp_err_to_name(err));
      }
    }
  }
}

static void on_rtsp_event(rtsp_event_t event, const rtsp_event_data_t *data,
                          void *user_data) {
  (void)data;
  (void)user_data;
  if (event != RTSP_EVENT_PLAYING || !s_auto_task) {
    return;
  }

  /* Nur beim Übergang in die Wiedergabe auslösen, und höchstens einmal pro
   * Minute. Ohne diese Sperre würde jedes Pause/Weiter die Sequenz erneut
   * anstoßen und den Eingang mitten im Hören umschalten. */
  static int64_t last_us = 0;
  int64_t now = esp_timer_get_time();
  if (last_us != 0 && now - last_us < 60LL * 1000 * 1000) {
    return;
  }
  last_us = now;

  /* Nicht im Callback senden: der läuft im RTSP-Task, und die Sequenz
   * dauert mit Wartezeit mehrere Sekunden. */
  xTaskNotifyGive(s_auto_task);
}

static void auto_input_start(void) {
  if (xTaskCreate(auto_input_task, "remote_auto", 3072, NULL, 4,
                  &s_auto_task) != pdPASS) {
    ESP_LOGW(TAG, "could not start the auto-input task");
    s_auto_task = NULL;
    return;
  }
  rtsp_events_register(on_rtsp_event, NULL);
  if (CONFIG_REMOTE_AUTO_INPUT_CMD[0] == '\0') {
    ESP_LOGI(TAG, "auto input: wake over RI only (input comes with it)");
  } else {
    ESP_LOGI(TAG, "auto input: wake, wait %d ms, then '%s'",
             CONFIG_REMOTE_AUTO_INPUT_DELAY_MS, CONFIG_REMOTE_AUTO_INPUT_CMD);
  }
}

#endif /* CONFIG_REMOTE_AUTO_INPUT */

/* ── Power sense: read the R-1045's own on/standby state off the RI line ──
 *
 * The RI tip carries a DC bias when the receiver is powered on and sits at
 * 0 V in standby — measured at the connector with a multimeter while the
 * RMT output idled, not inferred from a protocol. No frame, no timing, just
 * a level, so this needs the ADC rather than the RMT path used elsewhere in
 * this file.
 *
 * Two things make that harder than "just read the pin":
 *
 * 1. RMT drives the pin low between frames, which is exactly where the
 *    receiver's bias would show up. So each sample first releases the pin
 *    to a high-impedance input.
 *
 * 2. gpio_set_direction(..., OUTPUT) calls gpio_output_enable(), which
 *    resets the GPIO matrix to the plain-GPIO output signal — silently
 *    disconnecting the RMT channel from the pin for good. So the matrix
 *    routing is saved before the release and put back verbatim afterwards;
 *    without that, the first sample would kill RI transmission.
 *
 * Releasing the pin also leaves the node to charge through whatever source
 * impedance the receiver presents, so the read needs settling time — an
 * immediate read after release sees the value the pin was just driven to
 * (0 V). How long is a measurement, not a guess: see the probe below.
 *
 * Everything here runs under s_lock, the mutex remote_control_send() takes,
 * so a sample never lands mid-frame and a frame never goes out mid-sample.
 */
#if CONFIG_REMOTE_POWER_SENSE

/* Write the pad back to the state captured in power_sense_start(). */
static void pad_restore(void) {
  gpio_set_pull_mode(s_ri_gpio, GPIO_FLOATING);
  esp_rom_gpio_pad_select_gpio(s_ri_gpio); /* IO_MUX back to the GPIO function */
  GPIO.func_out_sel_cfg[s_ri_gpio].val = s_pad_out_sel;
  if (s_pad_was_out) {
    GPIO.enable_w1ts = (1U << s_ri_gpio);
  }
}

/* Hold the lock, release the pin, wait `us` since release, read, repeat for
 * each entry of `at_us` (ascending, microseconds since release). */
static esp_err_t sense_read_series(const uint32_t *at_us, size_t n,
                                   remote_probe_point_t *out, int pull,
                                   int atten) {
  xSemaphoreTake(s_lock, portMAX_DELAY);

  /* 12 dB spans ~0-3.1 V but reads 0 below roughly 100 mV; 0 dB spans only
   * ~0-0.95 V and resolves the bottom of that range far better. Selecting it
   * re-runs gpio_config_as_analog() on the pad - harmless here, the pad is
   * restored below anyway. */
  const adc_atten_t at = atten == REMOTE_PROBE_ATTEN_0DB ? ADC_ATTEN_DB_0
                                                          : ADC_ATTEN_DB_12;
  adc_cali_handle_t cali =
      atten == REMOTE_PROBE_ATTEN_0DB ? s_adc_cali_0 : s_adc_cali_12;
  adc_oneshot_chan_cfg_t chan_cfg = {.atten = at,
                                     .bitwidth = ADC_BITWIDTH_DEFAULT};
  adc_oneshot_config_channel(s_adc, s_adc_channel, &chan_cfg);

  if (!s_sense_ext) {
    gpio_set_level(s_ri_gpio, 0); /* plain-GPIO output level, for the restore */
    gpio_set_direction(s_ri_gpio, GPIO_MODE_INPUT);
    gpio_set_pull_mode(s_ri_gpio, pull == REMOTE_PROBE_PULL_DOWN ? GPIO_PULLDOWN_ONLY
                                    : pull == REMOTE_PROBE_PULL_UP  ? GPIO_PULLUP_ONLY
                                                                    : GPIO_FLOATING);
  }

  esp_err_t err = ESP_OK;
  const int64_t t0 = esp_timer_get_time();
  for (size_t i = 0; i < n && err == ESP_OK; i++) {
    while (esp_timer_get_time() - t0 < (int64_t)at_us[i]) {
      if (at_us[i] - (esp_timer_get_time() - t0) > 3000) {
        vTaskDelay(1);
      } else {
        esp_rom_delay_us(50);
      }
    }
    int raw = 0, mv = 0;
    err = adc_oneshot_read(s_adc, s_adc_channel, &raw);
    if (err == ESP_OK) {
      adc_cali_raw_to_voltage(cali, raw, &mv);
    }
    out[i].t_us       = (uint32_t)(esp_timer_get_time() - t0);
    out[i].raw        = raw;
    out[i].millivolts = mv;
    out[i].level      = s_sense_ext ? 0 : gpio_get_level(s_ri_gpio);
  }

  if (!s_sense_ext) {
    pad_restore();
  }

  xSemaphoreGive(s_lock);
  return err;
}

esp_err_t remote_control_probe_ri(remote_probe_point_t *out, size_t *count,
                                  uint32_t step_us, int pull, int atten) {
  ESP_RETURN_ON_FALSE(out && count && s_adc, ESP_ERR_INVALID_STATE, TAG,
                      "power sense not running");

  /* step_us == 0: a roughly log-spaced schedule, so one release window shows
   * both a fast and a slow time constant (needs 7 points, floats the line
   * for 60 ms). Otherwise `*count` evenly spaced points. */
  static const uint32_t log_schedule[] = {0, 200, 1000, 3000, 10000, 30000,
                                          60000};
  if (step_us == 0) {
    const size_t n = sizeof(log_schedule) / sizeof(log_schedule[0]);
    ESP_RETURN_ON_FALSE(*count >= n, ESP_ERR_INVALID_SIZE, TAG, "buffer");
    esp_err_t err = sense_read_series(log_schedule, n, out, pull, atten);
    *count = n;
    return err;
  }

  const size_t n = *count;
  uint32_t *at = malloc(n * sizeof(*at));
  ESP_RETURN_ON_FALSE(at, ESP_ERR_NO_MEM, TAG, "schedule");
  for (size_t i = 0; i < n; i++) {
    at[i] = (uint32_t)i * step_us;
  }
  esp_err_t err = sense_read_series(at, n, out, pull, atten);
  free(at);
  return err;
}

/* 400 points 100 us apart = 40 ms, two whole mains periods at 50 Hz, so the
 * hum the released line picks up averages out instead of adding noise. */
#define SENSE_POINTS  400
#define SENSE_STEP_US 100

static remote_probe_point_t *s_sense_buf = NULL;

static void power_sense_task(void *arg) {
  (void)arg;
  const TickType_t interval =
      pdMS_TO_TICKS(CONFIG_REMOTE_POWER_SENSE_INTERVAL_MS);
  int  pending_votes = 0; /* consecutive readings disagreeing with s_sense.on */

  for (;;) {
    vTaskDelay(interval);

    size_t n = SENSE_POINTS;
    esp_err_t err = remote_control_probe_ri(s_sense_buf, &n, SENSE_STEP_US,
                                            REMOTE_PROBE_PULL_UP,
                                            REMOTE_PROBE_ATTEN_12DB);
    if (err != ESP_OK || n == 0) {
      ESP_LOGW(TAG, "power sense read failed: %s", esp_err_to_name(err));
      continue;
    }

    int64_t sum_raw = 0, sum_mv = 0;
    for (size_t i = 0; i < n; i++) {
      sum_raw += s_sense_buf[i].raw;
      sum_mv  += s_sense_buf[i].millivolts;
    }
    const float mv = (float)sum_mv / (float)n;
    /* The internal pull-up gives ~1.18 V; a receiver that is on loads the
     * line slightly more and pulls that down by ~2 mV. So "on" is BELOW the
     * threshold, not above it. */
    const bool below = mv * 1000.0f < (float)CONFIG_REMOTE_POWER_SENSE_THRESHOLD_UV;

    bool first = false;
    portENTER_CRITICAL(&s_sense_mux);
    first = !s_sense.valid;
    portEXIT_CRITICAL(&s_sense_mux);

    bool on = first ? below : s_sense.on;
    if (!first && below != s_sense.on) {
      /* Require two agreeing readings in a row before flipping, so one
       * noisy window does not toggle the reported state. */
      if (++pending_votes >= 2) {
        on = below;
        pending_votes = 0;
      }
    } else {
      pending_votes = 0;
    }

    portENTER_CRITICAL(&s_sense_mux);
    s_sense.valid      = true;
    s_sense.on         = on;
    s_sense.raw        = (int)(sum_raw / (int64_t)n);
    s_sense.millivolts = mv;
    portEXIT_CRITICAL(&s_sense_mux);
  }
}

static esp_err_t power_sense_start(int ri_gpio) {
  s_ri_gpio = ri_gpio;
  s_pad_out_sel = GPIO.func_out_sel_cfg[ri_gpio].val;
  s_pad_was_out = (GPIO.enable >> ri_gpio) & 1;
  s_sense_ext   = CONFIG_REMOTE_POWER_SENSE_GPIO >= 0;
  /* Dedicated pin if configured, else the RI pad itself. */
  const int adc_gpio = s_sense_ext ? CONFIG_REMOTE_POWER_SENSE_GPIO : ri_gpio;

  adc_oneshot_unit_init_cfg_t unit_cfg = {.unit_id = ADC_UNIT_1};
  adc_unit_t detected_unit;
  ESP_RETURN_ON_ERROR(
      adc_oneshot_io_to_channel(adc_gpio, &detected_unit, &s_adc_channel), TAG,
      "GPIO%d is not ADC-capable", adc_gpio);
  ESP_RETURN_ON_FALSE(detected_unit == ADC_UNIT_1, ESP_ERR_NOT_SUPPORTED, TAG,
                      "GPIO%d is on an ADC unit this code does not use",
                      adc_gpio);

  ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&unit_cfg, &s_adc), TAG,
                      "adc_oneshot_new_unit");

  /* 12 dB: the full range (~0-3.1 V), so a level pulled up or down through
   * the internal resistor still fits. */
  adc_oneshot_chan_cfg_t chan_cfg = {
      .atten    = ADC_ATTEN_DB_12,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
  };
  ESP_RETURN_ON_ERROR(
      adc_oneshot_config_channel(s_adc, s_adc_channel, &chan_cfg), TAG,
      "adc_oneshot_config_channel");
  /* On the RI pad that call switched the pad to analog and its output off -
   * undo it, RMT still needs to drive this pin. A dedicated pin is not
   * RMT's, so there is nothing to undo. */
  if (!s_sense_ext) {
    pad_restore();
  }

  adc_cali_curve_fitting_config_t cali_cfg = {
      .unit_id  = ADC_UNIT_1,
      .chan     = s_adc_channel,
      .atten    = ADC_ATTEN_DB_12,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
  };
  ESP_RETURN_ON_ERROR(
      adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_adc_cali_12), TAG,
      "adc_cali_create_scheme_curve_fitting (12 dB)");
  cali_cfg.atten = ADC_ATTEN_DB_0;
  ESP_RETURN_ON_ERROR(
      adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_adc_cali_0), TAG,
      "adc_cali_create_scheme_curve_fitting (0 dB)");

  s_sense_buf = malloc(SENSE_POINTS * sizeof(*s_sense_buf));
  ESP_RETURN_ON_FALSE(s_sense_buf, ESP_ERR_NO_MEM, TAG, "sense buffer");

  if (xTaskCreate(power_sense_task, "ri_power_sense", 3072, NULL, 3, NULL) !=
      pdPASS) {
    return ESP_ERR_NO_MEM;
  }

  ESP_LOGI(TAG,
           "power sense on GPIO%d (ADC1 ch%d%s): on below %d uV, every %d ms",
           adc_gpio, (int)s_adc_channel, s_sense_ext ? ", dedicated" : "", CONFIG_REMOTE_POWER_SENSE_THRESHOLD_UV,
           CONFIG_REMOTE_POWER_SENSE_INTERVAL_MS);
  return ESP_OK;
}

remote_power_sense_t remote_control_power_sense(void) {
  remote_power_sense_t snap;
  portENTER_CRITICAL(&s_sense_mux);
  snap = s_sense;
  portEXIT_CRITICAL(&s_sense_mux);
  return snap;
}

#else /* !CONFIG_REMOTE_POWER_SENSE */

remote_power_sense_t remote_control_power_sense(void) {
  return (remote_power_sense_t){.valid = false};
}

esp_err_t remote_control_probe_ri(remote_probe_point_t *out, size_t *count,
                                  uint32_t step_us, int pull, int atten) {
  (void)atten;
  (void)pull;
  (void)out;
  (void)count;
  (void)step_us;
  return ESP_ERR_NOT_SUPPORTED;
}

#endif /* CONFIG_REMOTE_POWER_SENSE */

esp_err_t remote_control_send(const char *name, int repeat) {
  ESP_RETURN_ON_FALSE(name, ESP_ERR_INVALID_ARG, TAG, "no command");
  ESP_RETURN_ON_FALSE(remote_control_available(), ESP_ERR_INVALID_STATE, TAG,
                      "remote control not configured");

  size_t count = 0;
  const remote_command_t *cmds = remote_control_commands(&count);
  const remote_command_t *found = NULL;
  for (size_t i = 0; i < count; i++) {
    if (strcasecmp(name, cmds[i].name) == 0) {
      found = &cmds[i];
      break;
    }
  }
  ESP_RETURN_ON_FALSE(found, ESP_ERR_NOT_FOUND, TAG, "unknown command '%s'",
                      name);

  if (repeat < 1) {
    repeat = 1;
  }
  if (repeat > 20) {
    repeat = 20; /* a stuck client should not hold the bus for minutes */
  }

  xSemaphoreTake(s_lock, portMAX_DELAY);
  esp_err_t err = ESP_OK;
  for (int i = 0; i < repeat && err == ESP_OK; i++) {
    err = found->is_ir ? remote_control_send_ir(found->address, found->command)
                       : remote_control_send_ri(found->command);
  }
  xSemaphoreGive(s_lock);

  ESP_LOGI(TAG, "sent %s x%d -> %s", found->name, repeat, esp_err_to_name(err));
  return err;
}
