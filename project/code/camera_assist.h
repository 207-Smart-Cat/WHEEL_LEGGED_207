#ifndef CAMERA_ASSIST_H
#define CAMERA_ASSIST_H

#include "camera_align.h"
#include "zf_common_typedef.h"
#include "zf_device_mt9v03x.h"

#define CAMERA_ASSIST_ROI_X_LEFT    (8U)
#define CAMERA_ASSIST_ROI_X_RIGHT   (MT9V03X_W - 9U)
#define CAMERA_ASSIST_ROI_Y_TOP     (18U)
#define CAMERA_ASSIST_ROI_Y_BOTTOM  (MT9V03X_H - 26U)
#define CAMERA_ASSIST_EXPOSURE_MIN             (0U)
#define CAMERA_ASSIST_EXPOSURE_FINE_THRESHOLD  (40U)
#define CAMERA_ASSIST_EXPOSURE_MEDIUM_THRESHOLD (100U)
#define CAMERA_ASSIST_EXPOSURE_MAX             (1000U)

typedef enum
{
    CAMERA_EXPOSURE_SAVE_DEFAULT = 0,
    CAMERA_EXPOSURE_SAVE_PENDING,
    CAMERA_EXPOSURE_SAVE_SAVED,
    CAMERA_EXPOSURE_SAVE_ERROR
} CameraExposureSaveState_t;

typedef enum
{
    CAMERA_ASSIST_MODE_CENTER = 0,
    CAMERA_ASSIST_MODE_BRIDGE,
    CAMERA_ASSIST_MODE_STEP
} CameraAssistMode_t;

typedef struct
{
    uint32 frame_count;
    int16 lane_center_x;
    int16 lane_error_px;
    int16 raw_lane_error_px;
    int16 heading_error_px;
    float vision_angle_raw_deg;
    float vision_angle_offset_deg;
    uint16 exposure_time;
    uint8 threshold;
    uint8 valid_rows;
    uint8 dark_ratio_pct;
    uint8 ready;
    uint8 lane_valid;
    uint8 mode;
    uint8 exposure_save_state;
} CameraAssistStatus_t;

extern CameraAssistStatus_t camera_assist_status;

void CameraAssist_Init(void);
void CameraAssist_AttachToInitializedCamera(void);
void CameraAssist_ResetVisionControl(void);
void CameraAssist_ProcessFrame(void);
void CameraAssist_ConfigTask(void);
void CameraAssist_SetMode(CameraAssistMode_t mode);
uint8 CameraAssist_SetExposureTime(uint16 exposure_time);
void CameraAssist_RequestExposureSave(void);
uint8 CameraAssist_FlushExposureConfig(void);
uint8 CameraAssist_GetLateralError(float *error_px);
uint8 CameraAssist_GetLaneBounds(uint16 row, uint8 *left_x, uint8 *right_x);
const uint8 *CameraAssist_GetFrameImage(void);

#endif
