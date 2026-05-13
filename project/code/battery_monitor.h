#ifndef CODE_BATTERY_MONITOR_H_
#define CODE_BATTERY_MONITOR_H_

#include "zf_common_headfile.h"

// Seekfree motherboard battery voltage detect circuit.
#define BATTERY_ADC_CHANNEL          ADC0_CH21_P07_5
#define BATTERY_ADC_SCALE            (36.3f / 4096.0f)
#define BATTERY_FILTER_ALPHA         (0.12f)
#define BATTERY_LOW_THRESHOLD_V      (11.0f)
#define BATTERY_VALID_MIN_V          (1.0f)

extern float battery_voltage;
extern uint16 battery_adc_raw;

void battery_monitor_init(void);
void battery_monitor_update(void);
bool battery_monitor_is_low(void);

#endif
