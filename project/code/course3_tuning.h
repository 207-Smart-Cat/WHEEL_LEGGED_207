#ifndef COURSE3_TUNING_H
#define COURSE3_TUNING_H

/* Course 3 vision alignment thresholds. */
#define VISION_ALIGN_STABLE_WINDOW_FRAMES  (20U)
#define VISION_ALIGN_SAMPLE_COUNT_TARGET   (20U)
#define VISION_ALIGN_SIDE_SAMPLE_TARGET    (10U)
#define VISION_ALIGN_STABLE_ERROR_PX       (40)
#define VISION_ALIGN_SAMPLE_ERROR_PX       (5)

/* Course 3 pixel-error to yaw-angle PD mapping. */
#define VISION_PIXEL_TO_ANGLE_P_DEG_PER_PX (0.42f)
#define VISION_PIXEL_TO_ANGLE_D_DEG_PER_PX (0.20f)
#define VISION_DEADBAND_PX                 (0)
#define VISION_FILTER_OLD                  (0.75f)
#define VISION_FILTER_NEW                  (0.25f)
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
#define COURSE3_VISION_CAL_SPEED           (50.0f)

/* Course 3 action execution parameters. */
#define COURSE3_BRIDGE_SPEED               (233.0f)
#define COURSE3_BRIDGE_LEG_Y               (0.03f)
#define COURSE3_BRIDGE_SPEED_P             (0.012f)
#define COURSE3_BRIDGE_HOLD_MS             (1000U)
#define COURSE3_VISION_DIRECTION_P         (30.0f)
#define COURSE3_ACTION_DIRECTION_P         (50.0f)
#define COURSE3_BRIDGE_DIRECTION_P         (50.0f)
#define COURSE3_AUX_SEGMENT_SPEED          (300.0f)
#define COURSE3_AUX_SEGMENT_DIRECTION_P    (50.0f)

#endif
