# CM7_1 Vision Image Migration Design

## Goal

Migrate the non-special-element vision pipeline from the reference `image.c` and `image.h` into this repository so CM7_1 performs MT9V03X capture, image processing, and IPS200 display. CM7_0 remains unchanged and no control behavior is connected in this phase.

## Scope

The migration includes the basic visual processing path:

- MT9V03X grayscale frame copy into a local processing buffer.
- Otsu threshold calculation.
- Binary image generation.
- Morphological noise filtering.
- Black border preprocessing.
- Bottom-row left and right start-point detection.
- Eight-neighborhood boundary tracing.
- Per-row left boundary, right boundary, and center-line extraction.
- Weighted center output for future control use.
- IPS200 grayscale image display with overlays for left boundary, right boundary, algorithm center line, and fixed display reference center line.

The migration excludes all special-element detection and control side effects:

- Crossroad detection and filling.
- Roundabout detection and filling.
- Single-side bridge detection and filling.
- Zebra crossing stop detection.
- Jump detection.
- Buzzer, motor, velocity, PIT, and emergency control writes.

## Architecture

Create a focused CM7_1 application module in `project/code/vision_image.c` with public declarations in `project/code/vision_image.h`. The module owns all frame buffers, processing state, and result data. Consumers use `vision_image_get_result()` and buffer accessors instead of reading internal globals directly.

CM7_1 initializes the camera and vision module in `project/user/main_cm7_1.c`. The main loop consumes new frames using `mt9v03x_finish_flag`, processes `mt9v03x_image`, and clears the finish flag. The screen UI reads the latest result and displays it on a new `Vision` page.

## Data Interface

`VisionImageResult_t` is the future control-facing interface:

```c
typedef struct {
    uint8 valid;
    uint8 threshold;
    uint8 highest;
    float center_weighted;
    uint8 left_boundary[MT9V03X_H];
    uint8 right_boundary[MT9V03X_H];
    uint8 center_line[MT9V03X_H];
} VisionImageResult_t;
```

`valid` is set when both bottom start points are found and boundary tracing has produced usable left and right borders. If no valid start point exists, the last buffers remain available for display, but `valid` becomes `0` and `center_weighted` is set to the fixed reference center `MT9V03X_W / 2`.

## Display

Add `UI_SCREEN_VISION` to the existing screen state machine in `project/code/screen_display.c`. Add a `Vision` item to the home menu. The page displays the grayscale source image with `ips200_show_gray_image()` at native `188 x 120` size.

The page overlays:

- Left boundary in blue.
- Right boundary in red.
- Algorithm center line in yellow.
- Fixed display reference center line in green at `x = MT9V03X_W / 2`, which is `94`.

Text diagnostics below the image show `valid`, `threshold`, `highest`, `center_weighted`, and the signed center error `center_weighted - 94`.

The `BACK` key returns to the home menu. `OK`, `UP`, and `DOWN` are reserved for later display-mode switching and do not change behavior in this phase.

## Build Integration

Add `vision_image.c` and `vision_image.h` to the CM7_1 IAR project files:

- `project/iar/project_config/cyt4bb7_cm_7_1.ewp`
- `project/iar/project_config/cyt4bb7_cm_7_1.ewt`

Do not add the new module to CM7_0 project files in this phase.

## Verification

Verification is static and integration-focused because this environment does not provide an IAR compiler. The implementation must pass:

- Text search proving excluded special-element functions are absent from `vision_image.c`.
- Text search proving `vision_image.c` does not reference control-layer symbols such as `target_velocity`, `pit_all_close`, `small_driver_set_duty`, `Encoder_Left`, or `Encoder_Right`.
- Symbol checks that `vision_image_process`, `vision_image_get_result`, `vision_image_get_original_buffer`, and `vision_image_get_binary_buffer` are declared and defined once.
- IAR project file checks that CM7_1 includes the new files and CM7_0 does not.
