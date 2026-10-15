/* Services a receiver backend offers to an on-device UI, and the entry points
 * every board UI provides. Shared with the Cardputer ADV / T-Dongle S3 port
 * (fedaykinofdune/CardputerADV-SDR); the original ESP32 backend
 * (targets/esp32/receiver.c) has snapshots only, no continuous IQ runs. */
#pragma once
#include <stdbool.h>
#include <stdint.h>

/* One burst of n packed I/Q words (I bits 0-9, Q bits 10-19, signed);
 * divider is the CAP rate index: 0 = 80, 1 = 40, 6 = 16 MS/s. NULL on failure.
 * The buffer stays valid until the next capture. */
const uint32_t *sdr_local_capture(unsigned n, unsigned divider);
unsigned sdr_local_max_samples(void);
unsigned sdr_local_freq(void);
void sdr_local_tune(unsigned mhz);
int sdr_local_gain(void); /* -1: hardware AGC */
unsigned sdr_local_gain_max(void);
void sdr_local_set_gain(int code);
unsigned sdr_local_bandwidth(void); /* MHz, 0: automatic */
void sdr_local_set_bandwidth(unsigned mhz);

/* Board UI. board_ui_init() runs before GPIO discovery so the board pins stay
 * reserved; board_ui_step() runs one frame in the command loop's idle slot
 * and returns false when it did nothing (the caller then yields). */
void board_ui_init(void);
bool board_ui_step(bool host_active);
/* Board diagnostics over the serial protocol; true when the line was one. */
bool board_ui_command(const char *line);
