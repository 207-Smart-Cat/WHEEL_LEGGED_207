#include "camera_assist.h"
#include "vision_control.h"

#include "cy_device_headers.h"
#include "zf_device_mt9v03x.h"
#include "zf_driver_flash.h"

#include <string.h>

#define CAMERA_ASSIST_TARGET_X          ((MT9V03X_W - 1U) / 2U)
#define CAMERA_ASSIST_MIN_LANE_WIDTH    (9U)
#define CAMERA_ASSIST_MAX_LANE_WIDTH    (168U)
#define CAMERA_ASSIST_MIN_VALID_ROWS    (12U)
#define CAMERA_ASSIST_THRESHOLD_OFFSET  (8U)
#define CAMERA_ASSIST_THRESHOLD_MIN     (80U)
#define CAMERA_ASSIST_THRESHOLD_MAX     (235U)
#define CAMERA_ASSIST_DARK_OFFSET       (28U)
#define CAMERA_CONFIG_MAGIC             (0x43414D45UL)
#define CAMERA_CONFIG_VERSION           (1UL)
#define CAMERA_CONFIG_PAGE_A            (93U)
#define CAMERA_CONFIG_PAGE_B            (94U)
#define CAMERA_CONFIG_NO_PAGE           (0xFFU)
#define CAMERA_CONFIG_WORD_COUNT        (5U)
#define CAMERA_CONFIG_SAVE_DELAY_TICKS  (1000U)
#define CAMERA_CONFIG_CHECK_XOR         (0xA55A3CC3UL)

typedef struct
{
    uint32 magic;
    uint32 version;
    uint32 sequence;
    uint32 exposure;
    uint32 checksum;
} CameraConfigRecord_t;

typedef char camera_config_record_must_be_20_bytes[
    (sizeof(CameraConfigRecord_t) == CAMERA_CONFIG_WORD_COUNT * sizeof(uint32)) ? 1 : -1];
typedef char camera_config_pages_must_be_reserved[
    (CAMERA_CONFIG_PAGE_A >= 80U && CAMERA_CONFIG_PAGE_B < 95U) ? 1 : -1];

CameraAssistStatus_t camera_assist_status;
static VisionControlState_t camera_vision_control;
static uint8 camera_config_active_page = CAMERA_CONFIG_NO_PAGE;
static uint32 camera_config_sequence;
static uint16 camera_config_save_delay;
static uint8 camera_config_dirty;

static uint8 camera_frame_snapshot[MT9V03X_H][MT9V03X_W];
static uint32 camera_histogram[256];
static uint8 camera_boundary_left[MT9V03X_H];
static uint8 camera_boundary_right[MT9V03X_H];
static uint8 camera_boundary_valid[MT9V03X_H];

static uint32 camera_config_checksum(const CameraConfigRecord_t *record)
{
    return record->magic ^ record->version ^ record->sequence ^
           record->exposure ^ CAMERA_CONFIG_CHECK_XOR;
}

static uint8 camera_config_read(uint32 page, CameraConfigRecord_t *record)
{
    flash_read_page_to_buffer(0U, page, CAMERA_CONFIG_WORD_COUNT);
    memcpy(record, flash_union_buffer, sizeof(*record));

    if (record->magic != CAMERA_CONFIG_MAGIC ||
        record->version != CAMERA_CONFIG_VERSION ||
        record->exposure < CAMERA_ASSIST_EXPOSURE_MIN ||
        record->exposure > CAMERA_ASSIST_EXPOSURE_MAX ||
        record->checksum != camera_config_checksum(record))
    {
        return 0U;
    }
    return 1U;
}

static uint8 camera_config_load(uint16 *exposure)
{
    CameraConfigRecord_t record_a;
    CameraConfigRecord_t record_b;
    uint8 valid_a = camera_config_read(CAMERA_CONFIG_PAGE_A, &record_a);
    uint8 valid_b = camera_config_read(CAMERA_CONFIG_PAGE_B, &record_b);
    const CameraConfigRecord_t *selected;

    if (!valid_a && !valid_b)
    {
        camera_config_active_page = CAMERA_CONFIG_NO_PAGE;
        camera_config_sequence = 0U;
        return 0U;
    }

    if (valid_a && valid_b)
    {
        selected = ((int32)(record_b.sequence - record_a.sequence) > 0) ?
                   &record_b : &record_a;
        camera_config_active_page = (selected == &record_b) ? 1U : 0U;
    }
    else if (valid_b)
    {
        selected = &record_b;
        camera_config_active_page = 1U;
    }
    else
    {
        selected = &record_a;
        camera_config_active_page = 0U;
    }

    camera_config_sequence = selected->sequence;
    *exposure = (uint16)selected->exposure;
    return 1U;
}

static uint8 camera_config_write(void)
{
    CameraConfigRecord_t record;
    CameraConfigRecord_t verify;
    uint32 target_page =
        (camera_config_active_page == 0U) ? CAMERA_CONFIG_PAGE_B : CAMERA_CONFIG_PAGE_A;
    uint8 target_index = (target_page == CAMERA_CONFIG_PAGE_B) ? 1U : 0U;
    uint32 interrupt_mask;

    memset(&record, 0, sizeof(record));
    record.magic = CAMERA_CONFIG_MAGIC;
    record.version = CAMERA_CONFIG_VERSION;
    record.sequence = camera_config_sequence + 1U;
    if (record.sequence == 0U)
    {
        record.sequence = 1U;
    }
    record.exposure = camera_assist_status.exposure_time;
    record.checksum = camera_config_checksum(&record);

    interrupt_mask = __get_PRIMASK();
    __disable_irq();
    flash_buffer_clear();
    memcpy(flash_union_buffer, &record, sizeof(record));
    flash_write_page_from_buffer(0U, target_page, CAMERA_CONFIG_WORD_COUNT);
    if (!interrupt_mask)
    {
        __enable_irq();
    }

    if (!camera_config_read(target_page, &verify) ||
        verify.sequence != record.sequence ||
        verify.exposure != record.exposure)
    {
        camera_config_dirty = 0U;
        camera_config_save_delay = 0U;
        camera_assist_status.exposure_save_state = CAMERA_EXPOSURE_SAVE_ERROR;
        return 0U;
    }

    camera_config_active_page = target_index;
    camera_config_sequence = record.sequence;
    camera_config_dirty = 0U;
    camera_config_save_delay = 0U;
    camera_assist_status.exposure_save_state = CAMERA_EXPOSURE_SAVE_SAVED;
    return 1U;
}

static void camera_reset_status(void)
{
    memset(&camera_assist_status, 0, sizeof(camera_assist_status));
    camera_assist_status.lane_center_x = CAMERA_ASSIST_TARGET_X;
    camera_assist_status.mode = CAMERA_ASSIST_MODE_CENTER;
    camera_assist_status.exposure_time = MT9V03X_EXP_TIME_DEF;
    camera_assist_status.exposure_save_state = CAMERA_EXPOSURE_SAVE_DEFAULT;
    CameraAssist_ResetVisionControl();
}

void CameraAssist_ResetVisionControl(void)
{
    vision_control_reset(&camera_vision_control);
    camera_assist_status.vision_angle_raw_deg = 0.0f;
    camera_assist_status.vision_angle_offset_deg = 0.0f;
}

static uint8 camera_capture_snapshot(void)
{
    uint32 interrupt_mask;

    if (!mt9v03x_finish_flag)
    {
        return 0U;
    }

    interrupt_mask = __get_PRIMASK();
    __disable_irq();
    if (!mt9v03x_finish_flag)
    {
        if (!interrupt_mask)
        {
            __enable_irq();
        }
        return 0U;
    }

    memcpy(camera_frame_snapshot[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);
    mt9v03x_finish_flag = 0U;
    __DMB();

    if (!interrupt_mask)
    {
        __enable_irq();
    }
    return 1U;
}

static uint8 camera_clamp_u8(uint16 value, uint8 min_value, uint8 max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return (uint8)value;
}

static uint8 camera_compute_otsu_threshold(void)
{
    uint16 x;
    uint16 y;
    uint16 level;
    uint32 total = 0;
    uint32 weighted_sum = 0;
    uint32 background_count = 0;
    uint32 background_sum = 0;
    float best_variance = -1.0f;
    uint8 best_threshold = 0;

    memset(camera_histogram, 0, sizeof(camera_histogram));
    for (y = CAMERA_ASSIST_ROI_Y_TOP; y <= CAMERA_ASSIST_ROI_Y_BOTTOM; y++)
    {
        for (x = CAMERA_ASSIST_ROI_X_LEFT; x <= CAMERA_ASSIST_ROI_X_RIGHT; x++)
        {
            camera_histogram[camera_frame_snapshot[y][x]]++;
            total++;
        }
    }

    for (level = 0; level < 256U; level++)
    {
        weighted_sum += (uint32)level * camera_histogram[level];
    }

    for (level = 0; level < 255U; level++)
    {
        uint32 foreground_count;
        float background_mean;
        float foreground_mean;
        float mean_delta;
        float variance;

        background_count += camera_histogram[level];
        if (background_count == 0U)
        {
            continue;
        }

        foreground_count = total - background_count;
        if (foreground_count == 0U)
        {
            break;
        }

        background_sum += (uint32)level * camera_histogram[level];
        background_mean = (float)background_sum / (float)background_count;
        foreground_mean = (float)(weighted_sum - background_sum) / (float)foreground_count;
        mean_delta = background_mean - foreground_mean;
        variance = (float)background_count * (float)foreground_count * mean_delta * mean_delta;
        if (variance > best_variance)
        {
            best_variance = variance;
            best_threshold = (uint8)level;
        }
    }

    return camera_clamp_u8((uint16)best_threshold + CAMERA_ASSIST_THRESHOLD_OFFSET,
                           CAMERA_ASSIST_THRESHOLD_MIN,
                           CAMERA_ASSIST_THRESHOLD_MAX);
}

static void camera_update_lane(uint8 threshold)
{
    uint16 x;
    uint16 y;
    uint16 valid_rows = 0;
    uint16 far_rows = 0;
    uint16 near_rows = 0;
    uint32 center_sum = 0;
    uint32 center_weight = 0;
    uint32 far_center_sum = 0;
    uint32 near_center_sum = 0;
    uint32 dark_count = 0;
    uint32 pixel_count = 0;
    uint8 dark_threshold = (threshold > CAMERA_ASSIST_DARK_OFFSET) ?
                           (uint8)(threshold - CAMERA_ASSIST_DARK_OFFSET) : 0U;

    memset(camera_boundary_valid, 0, sizeof(camera_boundary_valid));

    for (y = CAMERA_ASSIST_ROI_Y_TOP; y <= CAMERA_ASSIST_ROI_Y_BOTTOM; y++)
    {
        uint16 run_start = CAMERA_ASSIST_ROI_X_LEFT;
        uint16 first_start = 0;
        uint16 last_end = 0;
        uint16 run_width = 0;
        uint16 weight;
        uint16 lane_width;
        uint8 in_bright_run = 0;
        uint8 has_white_run = 0;

        for (x = CAMERA_ASSIST_ROI_X_LEFT; x <= CAMERA_ASSIST_ROI_X_RIGHT; x++)
        {
            uint8 pixel = camera_frame_snapshot[y][x];

            pixel_count++;
            if (pixel <= dark_threshold)
            {
                dark_count++;
            }

            if (pixel >= threshold)
            {
                if (!in_bright_run)
                {
                    run_start = x;
                    in_bright_run = 1;
                }
            }
            else if (in_bright_run)
            {
                run_width = (uint16)(x - run_start);
                if (run_width >= 3U)
                {
                    if (!has_white_run)
                    {
                        first_start = run_start;
                    }
                    last_end = (uint16)(x - 1U);
                    has_white_run = 1U;
                }
                in_bright_run = 0;
            }
        }

        if (in_bright_run)
        {
            run_width = (uint16)(CAMERA_ASSIST_ROI_X_RIGHT - run_start + 1U);
            if (run_width >= 3U)
            {
                if (!has_white_run)
                {
                    first_start = run_start;
                }
                last_end = CAMERA_ASSIST_ROI_X_RIGHT;
                has_white_run = 1U;
            }
        }

        lane_width = has_white_run ? (uint16)(last_end - first_start + 1U) : 0U;
        if (lane_width >= CAMERA_ASSIST_MIN_LANE_WIDTH &&
            lane_width <= CAMERA_ASSIST_MAX_LANE_WIDTH)
        {
            uint16 center_x = (uint16)((first_start + last_end) / 2U);

            weight = (uint16)(y - CAMERA_ASSIST_ROI_Y_TOP + 1U);
            center_sum += (uint32)center_x * weight;
            center_weight += weight;
            valid_rows++;
            camera_boundary_left[y] = (uint8)first_start;
            camera_boundary_right[y] = (uint8)last_end;
            camera_boundary_valid[y] = 1U;

            if (y <= (CAMERA_ASSIST_ROI_Y_TOP + 24U))
            {
                far_center_sum += center_x;
                far_rows++;
            }
            if (y >= (CAMERA_ASSIST_ROI_Y_BOTTOM - 24U))
            {
                near_center_sum += center_x;
                near_rows++;
            }
        }
    }

    camera_assist_status.threshold = threshold;
    camera_assist_status.valid_rows = (valid_rows > 255U) ? 255U : (uint8)valid_rows;
    camera_assist_status.dark_ratio_pct = (pixel_count == 0U) ? 0U :
        (uint8)((dark_count * 100U) / pixel_count);

    if (valid_rows >= CAMERA_ASSIST_MIN_VALID_ROWS && center_weight > 0U)
    {
        int16 raw_center = (int16)((center_sum + center_weight / 2U) / center_weight);
        int16 raw_lane_error_px = (int16)(raw_center - (int16)CAMERA_ASSIST_TARGET_X);

        if (!camera_assist_status.lane_valid)
        {
            camera_assist_status.lane_center_x = raw_center;
        }
        else
        {
            camera_assist_status.lane_center_x = (int16)
                ((3 * camera_assist_status.lane_center_x + raw_center + 2) / 4);
        }
        camera_assist_status.lane_error_px =
            (int16)(camera_assist_status.lane_center_x - (int16)CAMERA_ASSIST_TARGET_X);
        camera_assist_status.raw_lane_error_px = raw_lane_error_px;
        if (far_rows > 0U && near_rows > 0U)
        {
            camera_assist_status.heading_error_px = (int16)
                ((int16)(far_center_sum / far_rows) - (int16)(near_center_sum / near_rows));
        }
        else
        {
            camera_assist_status.heading_error_px = 0;
        }
        camera_assist_status.lane_valid = 1;
        camera_assist_status.vision_angle_offset_deg =
            vision_control_update(&camera_vision_control, camera_assist_status.lane_error_px);
        camera_assist_status.vision_angle_raw_deg = camera_vision_control.raw_offset_deg;
    }
    else
    {
        camera_assist_status.lane_valid = 0;
        camera_assist_status.raw_lane_error_px = 0;
        camera_assist_status.heading_error_px = 0;
        CameraAssist_ResetVisionControl();
    }
}

void CameraAssist_Init(void)
{
    uint16 saved_exposure;

    camera_config_dirty = 0U;
    camera_config_save_delay = 0U;
    camera_reset_status();
    camera_assist_status.ready = (mt9v03x_init() == 0U) ? 1U : 0U;
    if (camera_assist_status.ready && camera_config_load(&saved_exposure))
    {
        if (CameraAssist_SetExposureTime(saved_exposure) == 0U)
        {
            camera_assist_status.exposure_save_state = CAMERA_EXPOSURE_SAVE_SAVED;
        }
        else
        {
            camera_assist_status.exposure_save_state = CAMERA_EXPOSURE_SAVE_ERROR;
        }
    }
}

void CameraAssist_AttachToInitializedCamera(void)
{
    camera_reset_status();
    camera_assist_status.ready = 1U;
}

void CameraAssist_ProcessFrame(void)
{
    uint8 threshold;

    if (!camera_assist_status.ready || !camera_capture_snapshot())
    {
        return;
    }

    threshold = camera_compute_otsu_threshold();
    camera_update_lane(threshold);
    camera_assist_status.frame_count++;
}

void CameraAssist_ConfigTask(void)
{
    if (!camera_config_dirty)
    {
        return;
    }

    if (camera_config_save_delay > 0U)
    {
        camera_config_save_delay--;
        return;
    }

    (void)camera_config_write();
}

void CameraAssist_SetMode(CameraAssistMode_t mode)
{
    if (mode > CAMERA_ASSIST_MODE_STEP)
    {
        mode = CAMERA_ASSIST_MODE_CENTER;
    }
    camera_assist_status.mode = (uint8)mode;
}

uint8 CameraAssist_SetExposureTime(uint16 exposure_time)
{
    uint8 result;

    if (!camera_assist_status.ready ||
        exposure_time < CAMERA_ASSIST_EXPOSURE_MIN ||
        exposure_time > CAMERA_ASSIST_EXPOSURE_MAX)
    {
        return 1U;
    }

    result = mt9v03x_set_exposure_time(exposure_time);
    if (result == 0U)
    {
        camera_assist_status.exposure_time = exposure_time;
    }
    return result;
}

void CameraAssist_RequestExposureSave(void)
{
    camera_config_dirty = 1U;
    camera_config_save_delay = CAMERA_CONFIG_SAVE_DELAY_TICKS;
    camera_assist_status.exposure_save_state = CAMERA_EXPOSURE_SAVE_PENDING;
}

uint8 CameraAssist_FlushExposureConfig(void)
{
    if (!camera_config_dirty)
    {
        return 1U;
    }
    return camera_config_write();
}

uint8 CameraAssist_GetLateralError(float *error_px)
{
    if (error_px != 0)
    {
        *error_px = (float)camera_assist_status.lane_error_px;
    }
    return camera_assist_status.lane_valid;
}

uint8 CameraAssist_GetLaneBounds(uint16 row, uint8 *left_x, uint8 *right_x)
{
    if (row >= MT9V03X_H || !camera_boundary_valid[row])
    {
        return 0U;
    }

    if (left_x != 0)
    {
        *left_x = camera_boundary_left[row];
    }
    if (right_x != 0)
    {
        *right_x = camera_boundary_right[row];
    }
    return 1U;
}

const uint8 *CameraAssist_GetFrameImage(void)
{
    return (const uint8 *)camera_frame_snapshot;
}
