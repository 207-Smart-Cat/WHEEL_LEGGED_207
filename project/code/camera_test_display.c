#include "camera_test_display.h"

#include "camera_assist.h"
#include "ipc_shared_data.h"
#include "course3_display_state.h"
#include "course3_tuning.h"
#include "vision_align_calibration.h"
#include "zf_common_font.h"
#include "zf_device_ips200.h"
#include "zf_device_mt9v03x.h"

#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#define CAMERA_TEST_IMAGE_X          (26U)
#define CAMERA_TEST_IMAGE_Y          (196U)
#define CAMERA_TEST_IMAGE_WIDTH      (MT9V03X_W)
#define CAMERA_TEST_IMAGE_HEIGHT     (MT9V03X_H)
#define CAMERA_TEST_INFO_Y           (4U)
#define CAMERA_TEST_RENDER_INTERVAL  (3U)
#define CAMERA_TEST_EXPOSURE_MIN      (40U)
#define CAMERA_TEST_EXPOSURE_MAX      (1000U)
#define CAMERA_TEST_EXPOSURE_STEP     (25U)

static uint32 camera_test_last_frame;

static void camera_test_show_info(uint16 y, const char *text);
static const char *camera_test_vision_cal_state_text(uint8 state);

static const char *camera_test_turn_direction(float angle_error)
{
    if (angle_error > 0.05f) return "LEFT";
    if (angle_error < -0.05f) return "RIGHT";
    return "HOLD";
}

void CameraTestDisplay_DrawLaneBoundaries(void)
{
    uint16 y;

    for (y = CAMERA_ASSIST_ROI_Y_TOP; y <= CAMERA_ASSIST_ROI_Y_BOTTOM; y++)
    {
        uint8 left_x;
        uint8 right_x;

        if (!CameraAssist_GetLaneBounds(y, &left_x, &right_x))
        {
            continue;
        }

        ips200_draw_point(CAMERA_TEST_IMAGE_X + left_x,
                          CAMERA_TEST_IMAGE_Y + y,
                          RGB565_GREEN);
        ips200_draw_point(CAMERA_TEST_IMAGE_X + right_x,
                          CAMERA_TEST_IMAGE_Y + y,
                          RGB565_MAGENTA);
    }
}

void CameraTestDisplay_DrawCenterLines(void)
{
    uint16 y;
    uint16 image_center_x = CAMERA_TEST_IMAGE_X + (MT9V03X_W - 1U) / 2U;

    for (y = 0U; y < MT9V03X_H; y++)
    {
        ips200_draw_point(image_center_x,
                          CAMERA_TEST_IMAGE_Y + y,
                          RGB565_YELLOW);
    }

    if (camera_assist_status.lane_valid)
    {
        uint16 lane_x = CAMERA_TEST_IMAGE_X + (uint16)
            ((int16)((MT9V03X_W - 1U) / 2U) + camera_assist_status.raw_lane_error_px);

        for (y = CAMERA_ASSIST_ROI_Y_TOP; y <= CAMERA_ASSIST_ROI_Y_BOTTOM; y++)
        {
            ips200_draw_point(lane_x,
                              CAMERA_TEST_IMAGE_Y + y,
                              RGB565_RED);
        }
    }
}

void CameraTestDisplay_DrawStatusText(void)
{
    char line[32];
    float turn_error;
    const CameraAssistStatus_t *status = &camera_assist_status;

    if (status->frame_count == 0U || (status->frame_count % 5U) != 0U)
    {
        return;
    }

    sprintf(line, "Frame: %lu        ", (unsigned long)status->frame_count);
    camera_test_show_info(CAMERA_TEST_INFO_Y, line);
    sprintf(line, "Lane : %s       ", status->lane_valid ? "VALID" : "SEARCHING");
    camera_test_show_info(CAMERA_TEST_INFO_Y + 20U, line);
    sprintf(line, "Center: %3d px     ", status->lane_center_x);
    camera_test_show_info(CAMERA_TEST_INFO_Y + 40U, line);
    sprintf(line, "Err R:%3d F:%3d   ", status->raw_lane_error_px, status->lane_error_px);
    camera_test_show_info(CAMERA_TEST_INFO_Y + 60U, line);
    sprintf(line, "MapDeg:%6.2f     ", status->vision_angle_raw_deg);
    camera_test_show_info(CAMERA_TEST_INFO_Y + 80U, line);
    turn_error = status->vision_angle_offset_deg;
    sprintf(line, "Cmd:%-5s %6.2fdeg", camera_test_turn_direction(turn_error), fabsf(turn_error));
    camera_test_show_info(CAMERA_TEST_INFO_Y + 100U, line);
    sprintf(line, "Yaw:%7.2f T:%7.2f", core_a_status.yaw, core_a_status.target_angle_status);
    camera_test_show_info(CAMERA_TEST_INFO_Y + 120U, line);
    sprintf(line, "Trn L:%+5.0f R:%+5.0f", core_a_status.pid_out_turn, -core_a_status.pid_out_turn);
    camera_test_show_info(CAMERA_TEST_INFO_Y + 140U, line);
    sprintf(line, "PWM L:%5d R:%5d",
            abs((int)core_a_status.left_pwm_duty),
            abs((int)core_a_status.right_pwm_duty));
    camera_test_show_info(CAMERA_TEST_INFO_Y + 160U, line);
    sprintf(line, "V:%3.0f P:%3.1f I:%3.1f",
            core_a_status.target_velocity_status,
            COURSE3_VISION_DIRECTION_P,
            COURSE3_VISION_DIRECTION_I);
    camera_test_show_info(CAMERA_TEST_INFO_Y + 176U, line);
}

void CameraTestDisplay_DrawCourse3FsmOverlay(void)
{
    const char *text = Course3DisplayState_Text(core_a_status.course3_display_state);

    if (text == 0 || !core_a_status.course3_exec_active)
    {
        return;
    }

    ips200_set_color(RGB565_RED, RGB565_BLACK);
    ips200_set_font(IPS200_16X16_FONT);
    ips200_show_string(8U, 164U, text);
    ips200_set_font(IPS200_8X16_FONT);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
}

static void camera_test_show_info(uint16 y, const char *text)
{
    ips200_show_string(8, y, text);
}

static const char *camera_test_vision_cal_state_text(uint8 state)
{
    switch (state)
    {
        case VISION_ALIGN_CAL_IDLE:        return "IDLE";
        case VISION_ALIGN_CAL_ALIGNING:    return "ALIGNING";
        case VISION_ALIGN_CAL_STABLE_WAIT: return "STABLE_WAIT";
        case VISION_ALIGN_CAL_SAMPLING:    return "SAMPLING";
        case VISION_ALIGN_CAL_DONE:        return "DONE";
        default:                           return "UNKNOWN";
    }
}

static void camera_test_draw_layout(void)
{
    ips200_clear();
    ips200_draw_line(0, 188, 239, 188, RGB565_CYAN);
    ips200_draw_line(0, 318, 239, 318, RGB565_CYAN);
}

void CameraTestDisplay_ResetRenderState(void)
{
    camera_test_last_frame = 0U;
    camera_test_draw_layout();
    camera_test_show_info(CAMERA_TEST_INFO_Y, "Camera: starting...");
}

void CameraTestDisplay_AdjustExposure(int16 delta)
{
    uint16 exposure = camera_assist_status.exposure_time;

    if (delta < 0)
    {
        if (exposure > (CAMERA_TEST_EXPOSURE_MIN + CAMERA_TEST_EXPOSURE_STEP))
        {
            exposure = (uint16)(exposure - CAMERA_TEST_EXPOSURE_STEP);
        }
        else
        {
            exposure = CAMERA_TEST_EXPOSURE_MIN;
        }
    }
    else if (exposure < (CAMERA_TEST_EXPOSURE_MAX - CAMERA_TEST_EXPOSURE_STEP))
    {
        exposure = (uint16)(exposure + CAMERA_TEST_EXPOSURE_STEP);
    }
    else
    {
        exposure = CAMERA_TEST_EXPOSURE_MAX;
    }

    if (CameraAssist_SetExposureTime(exposure) == 0U)
    {
        mt9v03x_finish_flag = 0U;
        camera_test_last_frame = camera_assist_status.frame_count;
    }
}

void CameraTestDisplay_Render(void)
{
    char line[32];
    uint16 image_center_x;
    uint16 lane_x;
    float turn_error;
    const CameraAssistStatus_t *status = &camera_assist_status;

    if (!status->ready)
    {
        camera_test_show_info(CAMERA_TEST_INFO_Y, "Camera: OFFLINE          ");
        return;
    }

    if (status->frame_count == 0U ||
        (status->frame_count - camera_test_last_frame) < CAMERA_TEST_RENDER_INTERVAL)
    {
        return;
    }
    camera_test_last_frame = status->frame_count;

    ips200_show_gray_image(CAMERA_TEST_IMAGE_X, CAMERA_TEST_IMAGE_Y,
                           CameraAssist_GetFrameImage(),
                           MT9V03X_W, MT9V03X_H,
                           CAMERA_TEST_IMAGE_WIDTH, CAMERA_TEST_IMAGE_HEIGHT, 0U);

    image_center_x = CAMERA_TEST_IMAGE_X + (MT9V03X_W - 1U) / 2U;
    ips200_draw_line(image_center_x, CAMERA_TEST_IMAGE_Y,
                     image_center_x, CAMERA_TEST_IMAGE_Y + MT9V03X_H - 1U,
                     RGB565_YELLOW);
    ips200_draw_line(CAMERA_TEST_IMAGE_X + CAMERA_ASSIST_ROI_X_LEFT,
                     CAMERA_TEST_IMAGE_Y + CAMERA_ASSIST_ROI_Y_TOP,
                     CAMERA_TEST_IMAGE_X + CAMERA_ASSIST_ROI_X_RIGHT,
                     CAMERA_TEST_IMAGE_Y + CAMERA_ASSIST_ROI_Y_TOP,
                     RGB565_CYAN);
    CameraTestDisplay_DrawLaneBoundaries();

    if (status->lane_valid)
    {
        lane_x = CAMERA_TEST_IMAGE_X + (uint16)
            ((int16)((MT9V03X_W - 1U) / 2U) + status->raw_lane_error_px);
        ips200_draw_line(lane_x, CAMERA_TEST_IMAGE_Y + CAMERA_ASSIST_ROI_Y_TOP,
                         lane_x, CAMERA_TEST_IMAGE_Y + CAMERA_ASSIST_ROI_Y_BOTTOM,
                         RGB565_RED);
    }

    sprintf(line, "Frame: %lu        ", (unsigned long)status->frame_count);
    camera_test_show_info(CAMERA_TEST_INFO_Y, line);
    sprintf(line, "Lane : %s       ", status->lane_valid ? "VALID" : "SEARCHING");
    camera_test_show_info(CAMERA_TEST_INFO_Y + 20U, line);
    sprintf(line, "Err R:%3d F:%3d", status->raw_lane_error_px, status->lane_error_px);
    camera_test_show_info(CAMERA_TEST_INFO_Y + 40U, line);
    turn_error = status->vision_angle_offset_deg;
    sprintf(line, "Cmd:%-5s %6.2fdeg", camera_test_turn_direction(turn_error), fabsf(turn_error));
    camera_test_show_info(CAMERA_TEST_INFO_Y + 60U, line);
    sprintf(line, "Yaw:%7.2f T:%7.2f", core_a_status.yaw, core_a_status.target_angle_status);
    camera_test_show_info(CAMERA_TEST_INFO_Y + 80U, line);
    sprintf(line, "Trn L:%+5.0f R:%+5.0f", core_a_status.pid_out_turn, -core_a_status.pid_out_turn);
    camera_test_show_info(CAMERA_TEST_INFO_Y + 100U, line);
    sprintf(line, "PWM L:%5d R:%5d",
            abs((int)core_a_status.left_pwm_duty),
            abs((int)core_a_status.right_pwm_duty));
    camera_test_show_info(CAMERA_TEST_INFO_Y + 120U, line);
    sprintf(line, "Cal:%-11s R:%5.1f%s",
            camera_test_vision_cal_state_text(core_a_status.vision_align_cal_state),
            core_a_status.vision_align_result_yaw,
            core_a_status.vision_align_result_valid ? "OK" : "--");
    camera_test_show_info(CAMERA_TEST_INFO_Y + 140U, line);
    sprintf(line, "V:%3.0f P:%3.1f I:%3.1f",
            core_a_status.target_velocity_status,
            COURSE3_VISION_DIRECTION_P,
            COURSE3_VISION_DIRECTION_I);
    camera_test_show_info(CAMERA_TEST_INFO_Y + 160U, line);
    sprintf(line, "D:%3.1f PxP:%4.2f PxD:%4.2f",
            COURSE3_VISION_DIRECTION_D,
            VISION_PIXEL_TO_ANGLE_P_DEG_PER_PX,
            VISION_PIXEL_TO_ANGLE_D_DEG_PER_PX);
    camera_test_show_info(CAMERA_TEST_INFO_Y + 176U, line);
}
