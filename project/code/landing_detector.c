#include "landing_detector.h"

#include <math.h>
#include <stddef.h>

#define LANDING_BASELINE_MIN_G          (0.60f)
#define LANDING_BASELINE_MAX_G          (1.40f)
#define LANDING_AIRBORNE_RATIO          (0.65f)
#define LANDING_IMPACT_DELTA_G          (0.50f)
#define LANDING_SETTLE_BAND_G           (0.35f)
#define LANDING_SOFT_BAND_G             (0.25f)
#define LANDING_SOFT_MIN_TICKS          (16U)
#define LANDING_TIMEOUT_TICKS           (160U)

static uint8_t landing_count_bits(uint8_t value)
{
    uint8_t count = 0U;

    while (value != 0U)
    {
        count = (uint8_t)(count + (value & 1U));
        value >>= 1U;
    }
    return count;
}

static uint8_t landing_in_band(float value, float center, float half_width)
{
    return (fabsf(value - center) <= half_width) ? 1U : 0U;
}

uint8_t LandingDetector_IsBaselineValid(float baseline_z_g)
{
    float magnitude;

    if (!isfinite(baseline_z_g))
    {
        return 0U;
    }
    magnitude = fabsf(baseline_z_g);
    return (magnitude >= LANDING_BASELINE_MIN_G &&
            magnitude <= LANDING_BASELINE_MAX_G) ? 1U : 0U;
}

void LandingDetector_Init(LandingDetector_t *detector, float baseline_z_g)
{
    if (detector == NULL)
    {
        return;
    }

    detector->state = LANDING_DETECTOR_WAIT_AIRBORNE;
    detector->baseline_g = fabsf(baseline_z_g);
    detector->axis_sign = (baseline_z_g < 0.0f) ? -1.0f : 1.0f;
    detector->elapsed_ticks = 0U;
    detector->started = 0U;
    detector->low_history = 0U;
    detector->low_samples = 0U;
    detector->impact_history = 0U;
    detector->impact_samples = 0U;
    detector->settle_history = 0U;
    detector->settle_samples = 0U;
    detector->soft_stable_count = 0U;
}

void LandingDetector_BeginAirborne(LandingDetector_t *detector)
{
    if (detector == NULL)
    {
        return;
    }

    detector->state = LANDING_DETECTOR_WAIT_AIRBORNE;
    detector->elapsed_ticks = 0U;
    detector->started = 1U;
    detector->low_history = 0U;
    detector->low_samples = 0U;
    detector->impact_history = 0U;
    detector->impact_samples = 0U;
    detector->settle_history = 0U;
    detector->settle_samples = 0U;
    detector->soft_stable_count = 0U;
}

LandingDetectorState_e LandingDetector_Update(LandingDetector_t *detector,
                                               float accel_z_g)
{
    float oriented_accel;
    uint8_t condition;

    if (detector == NULL)
    {
        return LANDING_DETECTOR_TIMEOUT;
    }
    if (!detector->started ||
        detector->state == LANDING_DETECTOR_LANDED ||
        detector->state == LANDING_DETECTOR_TIMEOUT)
    {
        return detector->state;
    }

    detector->elapsed_ticks++;
    if (detector->elapsed_ticks >= LANDING_TIMEOUT_TICKS)
    {
        detector->state = LANDING_DETECTOR_TIMEOUT;
        return detector->state;
    }

    oriented_accel = accel_z_g * detector->axis_sign;

    if (detector->state == LANDING_DETECTOR_WAIT_AIRBORNE)
    {
        condition = (oriented_accel <=
                     detector->baseline_g * LANDING_AIRBORNE_RATIO) ? 1U : 0U;
        detector->low_history = (uint8_t)(((detector->low_history << 1U) |
                                           condition) & 0x0FU);
        if (detector->low_samples < 4U)
        {
            detector->low_samples++;
        }
        if (detector->low_samples >= 4U &&
            landing_count_bits(detector->low_history) >= 3U)
        {
            detector->state = LANDING_DETECTOR_WAIT_IMPACT;
        }
        return detector->state;
    }

    if (detector->state == LANDING_DETECTOR_WAIT_IMPACT)
    {
        condition = (oriented_accel >=
                     detector->baseline_g + LANDING_IMPACT_DELTA_G) ? 1U : 0U;
        detector->impact_history = (uint8_t)(((detector->impact_history << 1U) |
                                              condition) & 0x0FU);
        if (detector->impact_samples < 4U)
        {
            detector->impact_samples++;
        }
        if (landing_count_bits(detector->impact_history) >= 2U)
        {
            detector->state = LANDING_DETECTOR_WAIT_SETTLE;
            detector->settle_history = 0U;
            detector->settle_samples = 0U;
            return detector->state;
        }

        if (detector->elapsed_ticks >= LANDING_SOFT_MIN_TICKS &&
            landing_in_band(oriented_accel,
                            detector->baseline_g,
                            LANDING_SOFT_BAND_G))
        {
            if (detector->soft_stable_count < 6U)
            {
                detector->soft_stable_count++;
            }
            if (detector->soft_stable_count >= 6U)
            {
                detector->state = LANDING_DETECTOR_LANDED;
            }
        }
        else
        {
            detector->soft_stable_count = 0U;
        }
        return detector->state;
    }

    condition = landing_in_band(oriented_accel,
                                detector->baseline_g,
                                LANDING_SETTLE_BAND_G);
    detector->settle_history = (uint8_t)(((detector->settle_history << 1U) |
                                          condition) & 0x1FU);
    if (detector->settle_samples < 5U)
    {
        detector->settle_samples++;
    }
    if (detector->settle_samples >= 5U &&
        landing_count_bits(detector->settle_history) >= 4U)
    {
        detector->state = LANDING_DETECTOR_LANDED;
    }
    return detector->state;
}

LandingDetectorState_e LandingDetector_GetState(const LandingDetector_t *detector)
{
    if (detector == NULL)
    {
        return LANDING_DETECTOR_TIMEOUT;
    }
    return detector->state;
}
