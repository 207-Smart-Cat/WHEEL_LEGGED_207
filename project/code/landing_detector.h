#ifndef LANDING_DETECTOR_H
#define LANDING_DETECTOR_H

#include <stdint.h>

typedef enum
{
    LANDING_DETECTOR_WAIT_AIRBORNE = 0,
    LANDING_DETECTOR_WAIT_IMPACT,
    LANDING_DETECTOR_WAIT_SETTLE,
    LANDING_DETECTOR_LANDED,
    LANDING_DETECTOR_TIMEOUT
} LandingDetectorState_e;

typedef struct
{
    LandingDetectorState_e state;
    float baseline_g;
    float axis_sign;
    uint16_t elapsed_ticks;
    uint8_t started;
    uint8_t low_history;
    uint8_t low_samples;
    uint8_t impact_history;
    uint8_t impact_samples;
    uint8_t settle_history;
    uint8_t settle_samples;
    uint8_t soft_stable_count;
} LandingDetector_t;

uint8_t LandingDetector_IsBaselineValid(float baseline_z_g);
void LandingDetector_Init(LandingDetector_t *detector, float baseline_z_g);
void LandingDetector_BeginAirborne(LandingDetector_t *detector);
LandingDetectorState_e LandingDetector_Update(LandingDetector_t *detector,
                                               float accel_z_g);
LandingDetectorState_e LandingDetector_GetState(const LandingDetector_t *detector);

#endif
