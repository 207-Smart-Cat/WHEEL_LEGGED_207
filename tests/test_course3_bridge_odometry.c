#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "course3_bridge_odometry.h"

static void assert_close(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.0001f);
}

int main(void)
{
    Course3BridgeOdometry_t odometry;
    float dx;
    float dy;
    Course3TravelMeter_t meter;

    assert_close(Course3Bridge_ComputeLength(0.0f, 0.0f, 3.0f, 4.0f), 5.0f);
    assert_close(Course3Bridge_MinWheelSpeed(0.50f, -0.30f), 0.30f);

    Course3BridgeOdometry_Begin(&odometry,
                                 0.0f,
                                 0.0f, 0.0f,
                                 3.0f, 4.0f);
    assert_close(odometry.target_distance_m, 5.0f);

    assert(Course3BridgeOdometry_Update(&odometry, 0.50f, -0.30f, 0.50f, &dx, &dy) == 0U);
    assert_close(dx, 0.15f);
    assert_close(dy, 0.0f);
    assert_close(odometry.travelled_distance_m, 0.15f);

    Course3BridgeOdometry_Begin(&odometry,
                                 90.0f,
                                 0.0f, 0.0f,
                                 1.0f, 0.0f);
    assert(Course3BridgeOdometry_Update(&odometry, 1.0f, -1.0f, 0.90f, &dx, &dy) == 0U);
    assert_close(dx, 0.0f);
    assert_close(dy, 0.90f);

    assert(Course3BridgeOdometry_Update(&odometry, 1.0f, -1.0f, 0.10f, &dx, &dy) == 1U);
    assert_close(dx, 0.0f);
    assert_close(dy, 0.10f);
    assert_close(odometry.travelled_distance_m, 1.00f);

    Course3TravelMeter_Begin(&meter, 0.50f);
    assert(Course3TravelMeter_Update(&meter, 0.30f, -0.20f, 1.0f) == 0U);
    assert_close(meter.travelled_distance_m, 0.20f);
    assert(Course3TravelMeter_Update(&meter, 0.50f, -0.40f, 1.0f) == 1U);
    assert_close(meter.travelled_distance_m, 0.50f);

    puts("course3_bridge_odometry tests passed");
    return 0;
}
