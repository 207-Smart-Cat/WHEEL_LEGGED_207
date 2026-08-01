#include <assert.h>
#include <stdint.h>

#include "landing_detector.h"

static void feed(LandingDetector_t *detector, float accel_z_g, uint32_t count)
{
    uint32_t i;

    for (i = 0U; i < count; ++i)
    {
        (void)LandingDetector_Update(detector, accel_z_g);
    }
}

static void qualify_airborne(LandingDetector_t *detector, float accel_z_g)
{
    LandingDetector_BeginAirborne(detector);
    feed(detector, accel_z_g, 4U);
    assert(LandingDetector_GetState(detector) == LANDING_DETECTOR_WAIT_IMPACT);
}

static void test_rejects_invalid_gravity_baseline(void)
{
    assert(!LandingDetector_IsBaselineValid(0.59f));
    assert(LandingDetector_IsBaselineValid(0.60f));
    assert(LandingDetector_IsBaselineValid(1.40f));
    assert(!LandingDetector_IsBaselineValid(1.41f));
    assert(LandingDetector_IsBaselineValid(-1.00f));
}

static void test_takeoff_impulse_cannot_land_before_airborne(void)
{
    LandingDetector_t detector;

    LandingDetector_Init(&detector, 1.0f);
    feed(&detector, 1.8f, 8U);
    assert(LandingDetector_GetState(&detector) == LANDING_DETECTOR_WAIT_AIRBORNE);
}

static void test_isolated_impact_does_not_confirm_landing(void)
{
    LandingDetector_t detector;

    LandingDetector_Init(&detector, 1.0f);
    qualify_airborne(&detector, 0.20f);
    (void)LandingDetector_Update(&detector, 1.70f);
    feed(&detector, 0.20f, 4U);
    assert(LandingDetector_GetState(&detector) == LANDING_DETECTOR_WAIT_IMPACT);
}

static void test_hard_impact_requires_settle(void)
{
    LandingDetector_t detector;

    LandingDetector_Init(&detector, 1.0f);
    qualify_airborne(&detector, 0.20f);
    (void)LandingDetector_Update(&detector, 1.70f);
    (void)LandingDetector_Update(&detector, 1.65f);
    assert(LandingDetector_GetState(&detector) == LANDING_DETECTOR_WAIT_SETTLE);
    feed(&detector, 1.00f, 5U);
    assert(LandingDetector_GetState(&detector) == LANDING_DETECTOR_LANDED);
}

static void test_soft_landing_after_minimum_air_time(void)
{
    LandingDetector_t detector;

    LandingDetector_Init(&detector, 1.0f);
    qualify_airborne(&detector, 0.20f);
    feed(&detector, 0.20f, 12U);
    feed(&detector, 1.00f, 6U);
    assert(LandingDetector_GetState(&detector) == LANDING_DETECTOR_LANDED);
}

static void test_airborne_timeout_is_not_landing(void)
{
    LandingDetector_t detector;

    LandingDetector_Init(&detector, 1.0f);
    qualify_airborne(&detector, 0.20f);
    feed(&detector, 0.20f, 156U);
    assert(LandingDetector_GetState(&detector) == LANDING_DETECTOR_TIMEOUT);
}

static void test_negative_axis_orientation_is_normalized(void)
{
    LandingDetector_t detector;

    LandingDetector_Init(&detector, -1.0f);
    qualify_airborne(&detector, -0.20f);
    (void)LandingDetector_Update(&detector, -1.70f);
    (void)LandingDetector_Update(&detector, -1.65f);
    feed(&detector, -1.00f, 5U);
    assert(LandingDetector_GetState(&detector) == LANDING_DETECTOR_LANDED);
}

int main(void)
{
    test_rejects_invalid_gravity_baseline();
    test_takeoff_impulse_cannot_land_before_airborne();
    test_isolated_impact_does_not_confirm_landing();
    test_hard_impact_requires_settle();
    test_soft_landing_after_minimum_air_time();
    test_airborne_timeout_is_not_landing();
    test_negative_axis_orientation_is_normalized();
    return 0;
}
