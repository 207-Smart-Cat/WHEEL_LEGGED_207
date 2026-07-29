#include "camera_align.h"
#include "imu.h"

#include "zf_driver_gpio.h"

#define CAMERA_ALIGN_OK_PIN  P20_1

static CameraAlignLogic_t camera_align_logic;
static uint8 camera_align_active;
static float camera_align_hold_yaw;

void camera_align_reset(void)
{
    CameraAlignLogic_Reset(&camera_align_logic);
    camera_align_active = 0U;
    camera_align_hold_yaw = 0.0f;
}

uint8 camera_align(void)
{
    extern float target_angle;
    extern float target_velocity;

    if (!camera_align_active)
    {
        CameraAlignLogic_Reset(&camera_align_logic);
        camera_align_hold_yaw = IMU_data.filter_result.yaw;
        camera_align_active = 1U;
    }

    target_velocity = 0.0f;
    target_angle = camera_align_hold_yaw;

    if (CameraAlignLogic_Update(&camera_align_logic, gpio_get_level(CAMERA_ALIGN_OK_PIN)))
    {
        camera_align_reset();
        return 1U;
    }

    return 0U;
}
