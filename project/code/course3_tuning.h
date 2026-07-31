#ifndef COURSE3_TUNING_H
#define COURSE3_TUNING_H

/* Course 3 vision alignment thresholds. */
#define VISION_ALIGN_STABLE_WINDOW_FRAMES  (20U)
#define VISION_ALIGN_SAMPLE_COUNT_TARGET   (14U)
#define VISION_ALIGN_STABLE_ERROR_PX       (40)
#define VISION_ALIGN_SAMPLE_ERROR_PX       (5)
#define VISION_ALIGN_COMPLETE_TIMEOUT_MS   (800U)

/* Course 3 pixel-error to yaw-angle PD mapping. */
#define VISION_PIXEL_TO_ANGLE_P_DEG_PER_PX (0.3f)
#define VISION_PIXEL_TO_ANGLE_D_DEG_PER_PX (0.0f)
#define VISION_DEADBAND_PX                 (0)
#define VISION_MAX_ANGLE_OFFSET_DEG        (8.0f)
#define VISION_SIGN                        (-1.0f)

#define VISION_GAIN_DEG_PER_PX             VISION_PIXEL_TO_ANGLE_P_DEG_PER_PX
#define VISION_D_GAIN_DEG_PER_PX_STEP      VISION_PIXEL_TO_ANGLE_D_DEG_PER_PX

/* Course 3 action-line alignment. */
#define COURSE3_ALIGN_SIDE_SAMPLE_COUNT    (5U)
#define COURSE3_ALIGN_CANDIDATE_COUNT      (20U)
#define COURSE3_ALIGN_SPEED                (110.0f)
#define COURSE3_SEARCH_SPEED               (110.0f)
#define COURSE3_ALIGN_CENTER_PX            (5)
#define COURSE3_ALIGN_LOST_MS              (3000U)
#define VISION_MENU_CAL_SPEED              (120.0f)
#define COURSE3_VISION_CAL_SPEED           (120.0f)

/* Course 3 action execution parameters. */
#define COURSE3_BRIDGE_SPEED               (233.0f)
#define COURSE3_BRIDGE_LEG_Y               (0.03f)
#define COURSE3_BRIDGE_SPEED_P             (0.012f)
#define COURSE3_BRIDGE_HOLD_MS             (1000U)
#define COURSE3_VISION_YAW_RATE_SIGN       (-1.0f)
#define COURSE3_VISION_DIRECTION_P         (15.0f)
#define COURSE3_VISION_DIRECTION_I         (0.0f)
#define COURSE3_VISION_DIRECTION_D         (0.0f)
#define COURSE3_VISION_CONTROL_DT_S        (0.001f)
#define COURSE3_VISION_I_ERROR_MIN_DEG     (0.15f)
#define COURSE3_VISION_I_ERROR_MAX_DEG     (5.00f)
#define COURSE3_VISION_I_YAW_RATE_MAX_DPS  (8.0f)
#define COURSE3_VISION_I_OUTPUT_LIMIT      (80.0f)
#define COURSE3_VISION_TURN_PWM_LIMIT      (60.0f)
#define COURSE3_ACTION_DIRECTION_P         (50.0f)
#define COURSE3_BRIDGE_DIRECTION_P         (50.0f)
#define COURSE3_AUX_SEGMENT_SPEED          (300.0f)
#define COURSE3_BUMP_DIRECTION_P           (20.0f)
#define COURSE3_AUX_SEGMENT_DIRECTION_P    (50.0f)

#endif
