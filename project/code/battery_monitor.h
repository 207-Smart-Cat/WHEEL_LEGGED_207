#ifndef CODE_BATTERY_MONITOR_H_
#define CODE_BATTERY_MONITOR_H_

#include "zf_common_headfile.h"

// Change these three values to match the actual battery divider hardware.
#define BATTERY_ADC_CHANNEL          ADC0_CH00_P06_0
#define BATTERY_ADC_REF_VOLTAGE      (3.3f)
#define BATTERY_ADC_DIVIDER_RATIO    (11.0f)   // Example: 100k top, 10k bottom.
#define BATTERY_FILTER_ALPHA         (0.12f)

extern float battery_voltage;
extern uint16 battery_adc_raw;

void battery_monitor_init(void);
void battery_monitor_update(void);

#endif
