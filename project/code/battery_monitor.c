#include "battery_monitor.h"

float battery_voltage = 0.0f;
uint16 battery_adc_raw = 0;

static bool battery_monitor_ready = false;

void battery_monitor_init(void)
{
    adc_init(BATTERY_ADC_CHANNEL, ADC_12BIT);
    battery_adc_raw = adc_mean_filter_convert(BATTERY_ADC_CHANNEL, 8);
    battery_voltage = ((float)battery_adc_raw / 4095.0f) *
                      BATTERY_ADC_REF_VOLTAGE *
                      BATTERY_ADC_DIVIDER_RATIO;
    battery_monitor_ready = true;
}

void battery_monitor_update(void)
{
    float voltage;

    if (!battery_monitor_ready)
    {
        return;
    }

    battery_adc_raw = adc_mean_filter_convert(BATTERY_ADC_CHANNEL, 8);
    voltage = ((float)battery_adc_raw / 4095.0f) *
              BATTERY_ADC_REF_VOLTAGE *
              BATTERY_ADC_DIVIDER_RATIO;

    battery_voltage += BATTERY_FILTER_ALPHA * (voltage - battery_voltage);
}
