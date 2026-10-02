#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * Remote control for an Onkyo R-1045 over two buses.
 *
 * The receiver splits its remote functions across two incompatible paths, and
 * neither alone is enough:
 *
 *   RI  — a wired 3.5 mm bus. Carries exactly two useful commands, but both
 *         are *discrete*: 0x020 wakes the unit, 0x420 puts it in standby.
 *         Discrete matters, because the IR power key is a toggle and a toggle
 *         without feedback eventually turns the receiver on when an
 *         automation meant to turn it off.
 *
 *   IR  — everything else. Volume, mute and input selection exist only here;
 *         a full sweep of the 12-bit RI code space found no volume command,
 *         and the receiver ignores the documented ones at every input.
 *
 * Both buses are driven from the RMT peripheral rather than bit-banged. That
 * is not a stylistic choice: AirPlay 2 synchronises playback over PTP, and a
 * blocking software-timed routine holding the CPU for 60 ms would show up as
 * audible dropouts. RMT clocks the waveform in hardware.
 */

/** Initialise both transmitters. Pins come from Kconfig; a pin set to -1
 *  disables that bus. Safe to call when neither is configured. */
esp_err_t remote_control_init(void);

/** True when at least one bus is configured and ready. */
bool remote_control_available(void);

/** Send a named command from the table below, e.g. "volup" or "ri-on".
 *  Returns ESP_ERR_NOT_FOUND for an unknown name. */
esp_err_t remote_control_send(const char *name, int repeat);

/** Send an arbitrary NEC frame over IR. Address and command are in the same
 *  notation ESPHome's decoder reports, which is how the code book records
 *  them — no conversion needed. */
esp_err_t remote_control_send_ir(uint16_t address, uint16_t command);

/** Send an arbitrary 12-bit RI code. */
esp_err_t remote_control_send_ri(uint16_t code);

/** Command table, for building the /api/remote/list response. */
typedef struct {
  const char *name;
  bool        is_ir;
  uint16_t    address; /* IR only */
  uint16_t    command; /* IR command, or the 12-bit RI code */
  const char *label;
} remote_command_t;

const remote_command_t *remote_control_commands(size_t *count);

/** Result of the most recent RI-line power sense reading. `valid` is false
 *  until the first sample has been taken, or when CONFIG_REMOTE_POWER_SENSE
 *  is off. */
typedef struct {
  bool valid;
  bool on;
  int   raw;
  float millivolts; /* mean over the sample window, sub-mV resolution */
} remote_power_sense_t;

/** Last known receiver power state, from periodically sampling the DC bias
 *  on the RI line (see CONFIG_REMOTE_POWER_SENSE). Non-blocking - returns
 *  the most recent background sample, not a fresh reading. */
remote_power_sense_t remote_control_power_sense(void);

/** What holds the RI pad while it is released for a probe. */
enum { REMOTE_PROBE_PULL_NONE = 0, REMOTE_PROBE_PULL_DOWN = 1, REMOTE_PROBE_PULL_UP = 2 };

/** ADC attenuation for a probe: 12 dB spans ~0-3.1 V, 0 dB only ~0-0.95 V but
 *  resolves small levels near 0 V much better. */
enum { REMOTE_PROBE_ATTEN_12DB = 0, REMOTE_PROBE_ATTEN_0DB = 1 };

/** One reading taken during a probe: microseconds since the RI pin was
 *  released, the raw and calibrated ADC value, and the digital level the pad
 *  reads as at that moment. */
typedef struct {
  uint32_t t_us;
  int      raw;
  int      millivolts;
  int      level;
} remote_probe_point_t;

/** Diagnostic: release the RI pin once and read the ADC repeatedly, to see
 *  what the line does while nothing drives it. With `step_us == 0` a fixed
 *  roughly log-spaced schedule up to 60 ms is used (`*count` must be >= 7);
 *  otherwise `*count` points are taken `step_us` apart. `*count` returns the
 *  number of points. Holds the RI/IR bus, and leaves the line floating, for
 *  the whole window, held by `pull` (an internal ~45 kOhm resistor, or
 *  nothing). ESP_ERR_NOT_SUPPORTED without CONFIG_REMOTE_POWER_SENSE. */
esp_err_t remote_control_probe_ri(remote_probe_point_t *out, size_t *count,
                                  uint32_t step_us, int pull, int atten);
