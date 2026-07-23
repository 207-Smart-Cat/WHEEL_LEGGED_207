# CM7_1 Vision Image Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build CM7_1-only MT9V03X vision processing and IPS200 grayscale display with boundary, algorithm center, and fixed reference-center overlays.

**Architecture:** Add a focused `vision_image` module that owns image processing and exposes a stable result struct for future control integration. CM7_1 initializes and processes camera frames, while `screen_display.c` only renders the latest result through the new `Vision` UI page.

**Tech Stack:** C for CYT4BB7, SEEKFREE `zf_common_headfile.h`, MT9V03X camera driver, IPS200 display driver, IAR project files.

## Global Constraints

- CM7_1 is responsible for camera capture, vision processing, and IPS200 display.
- CM7_0 remains unchanged and receives no control integration in this phase.
- Migrate only basic vision processing, center-line logic, and display support.
- Exclude crossroad, roundabout, single-side bridge, zebra crossing, jump, buzzer, motor, velocity, PIT, and stop-control behavior.
- Display the grayscale image, not the binary image, and overlay left boundary, right boundary, algorithm center line, and fixed reference center line.
- Reference center line is `MT9V03X_W / 2`, equal to `94` for the current `188 x 120` MT9V03X configuration.

---

## File Structure

- Create `project/code/vision_image.h`: public constants, result struct, and processing/display data accessors.
- Create `project/code/vision_image.c`: Otsu thresholding, binary conversion, filtering, black frame, start-point search, eight-neighborhood tracing, per-row boundary extraction, weighted center calculation, and result storage.
- Modify `project/user/main_cm7_1.c`: initialize MT9V03X and the vision module, process finished camera frames in the CM7_1 main loop.
- Modify `project/code/app_headfile.h`: include `vision_image.h` so CM7_1 application files can use the module through the existing aggregate header.
- Modify `project/code/screen_display.c`: add `Vision` menu entry, state handling, and grayscale display with overlays.
- Modify `project/iar/project_config/cyt4bb7_cm_7_1.ewp`: add `vision_image.c` and `vision_image.h` to the CM7_1 app group.
- Modify `project/iar/project_config/cyt4bb7_cm_7_1.ewt`: add the same files to the CM7_1 template project.

### Task 1: Add Vision Module Interface

**Files:**
- Create: `project/code/vision_image.h`

**Interfaces:**
- Consumes: `MT9V03X_W`, `MT9V03X_H`, `uint8` from `zf_common_headfile.h`.
- Produces: `VisionImageResult_t`, `vision_image_init`, `vision_image_process`, `vision_image_get_result`, `vision_image_get_original_buffer`, `vision_image_get_binary_buffer`.

- [ ] **Step 1: Create the public header**

Add `project/code/vision_image.h`:

```c
#ifndef _VISION_IMAGE_H_
#define _VISION_IMAGE_H_

#include "zf_common_headfile.h"

#define VISION_IMAGE_H          (MT9V03X_H)
#define VISION_IMAGE_W          (MT9V03X_W)
#define VISION_WHITE_PIXEL      (255)
#define VISION_BLACK_PIXEL      (0)
#define VISION_BORDER_MIN       (1)
#define VISION_BORDER_MAX       (VISION_IMAGE_W - 2)
#define VISION_REFERENCE_CENTER (VISION_IMAGE_W / 2)

typedef struct {
    uint8 valid;
    uint8 threshold;
    uint8 highest;
    float center_weighted;
    uint8 left_boundary[VISION_IMAGE_H];
    uint8 right_boundary[VISION_IMAGE_H];
    uint8 center_line[VISION_IMAGE_H];
} VisionImageResult_t;

void vision_image_init(void);
void vision_image_process(const uint8 image[VISION_IMAGE_H][VISION_IMAGE_W]);
const VisionImageResult_t *vision_image_get_result(void);
const uint8 *vision_image_get_original_buffer(void);
const uint8 *vision_image_get_binary_buffer(void);

#endif
```

- [ ] **Step 2: Verify header guard and symbols**

Run:

```powershell
rg -n "VisionImageResult_t|vision_image_process|VISION_REFERENCE_CENTER" project\code\vision_image.h
```

Expected: the struct, process function, and reference-center macro are found once.

### Task 2: Implement Basic Vision Processing

**Files:**
- Create: `project/code/vision_image.c`

**Interfaces:**
- Consumes: `vision_image.h`.
- Produces: populated `VisionImageResult_t` and image buffers for UI rendering.

- [ ] **Step 1: Add buffers, helpers, and Otsu threshold**

Create `project/code/vision_image.c` with module-local state:

```c
#include "vision_image.h"

#define VISION_USE_NUM      (VISION_IMAGE_H * 3)
#define VISION_BIN_JUMP_NUM (1)
#define VISION_FILTER_MAX   (255 * 5)
#define VISION_FILTER_MIN   (255 * 2)

static uint8 original_image[VISION_IMAGE_H][VISION_IMAGE_W];
static uint8 binary_image[VISION_IMAGE_H][VISION_IMAGE_W];
static uint16 points_l[VISION_USE_NUM][2];
static uint16 points_r[VISION_USE_NUM][2];
static uint16 dir_l[VISION_USE_NUM];
static uint16 dir_r[VISION_USE_NUM];
static uint16 data_stastics_l;
static uint16 data_stastics_r;
static uint8 start_point_l[2];
static uint8 start_point_r[2];
static uint16 l_index[VISION_IMAGE_H];
static uint16 r_index[VISION_IMAGE_H];
static VisionImageResult_t vision_result;

static int vision_abs(int value)
{
    return (value >= 0) ? value : -value;
}

static int vision_limit_a_b(int x, int a, int b)
{
    if (x < a) x = a;
    if (x > b) x = b;
    return x;
}

static void vision_clear_result(void)
{
    for (uint8 i = 0; i < VISION_IMAGE_H; i++)
    {
        vision_result.left_boundary[i] = VISION_BORDER_MIN;
        vision_result.right_boundary[i] = VISION_BORDER_MAX;
        vision_result.center_line[i] = VISION_REFERENCE_CENTER;
        l_index[i] = 0;
        r_index[i] = 0;
    }
    vision_result.valid = 0;
    vision_result.threshold = 0;
    vision_result.highest = (uint8)(VISION_IMAGE_H - 2);
    vision_result.center_weighted = (float)VISION_REFERENCE_CENTER;
}
```

Then port the reference Otsu logic as a `static uint8 vision_otsu_threshold(const uint8 *image, uint16 col, uint16 row)` function. Use the same histogram and max-class-variance algorithm, but keep the function `static` and rename local identifiers to avoid exporting old names.

- [ ] **Step 2: Add frame copy, binary conversion, filtering, and black frame**

Implement these static functions:

```c
static void vision_copy_image(const uint8 image[VISION_IMAGE_H][VISION_IMAGE_W])
{
    for (uint8 y = 0; y < VISION_IMAGE_H; y++)
    {
        for (uint8 x = 0; x < VISION_IMAGE_W; x++)
        {
            original_image[y][x] = image[y][x];
        }
    }
}

static void vision_turn_to_bin(void)
{
    vision_result.threshold = vision_otsu_threshold(original_image[0], VISION_IMAGE_W, VISION_IMAGE_H);
    for (uint8 y = 0; y < VISION_IMAGE_H; y++)
    {
        for (uint8 x = 0; x < VISION_IMAGE_W; x++)
        {
            binary_image[y][x] = (original_image[y][x] > vision_result.threshold) ? VISION_WHITE_PIXEL : VISION_BLACK_PIXEL;
        }
    }
}

static void vision_filter(void)
{
    uint32 num;
    for (uint16 y = 1; y < VISION_IMAGE_H - 1; y++)
    {
        for (uint16 x = 1; x < VISION_IMAGE_W - 1; x++)
        {
            num = binary_image[y - 1][x - 1] + binary_image[y - 1][x] + binary_image[y - 1][x + 1]
                + binary_image[y][x - 1] + binary_image[y][x + 1]
                + binary_image[y + 1][x - 1] + binary_image[y + 1][x] + binary_image[y + 1][x + 1];
            if (num >= VISION_FILTER_MAX && binary_image[y][x] == VISION_BLACK_PIXEL) binary_image[y][x] = VISION_WHITE_PIXEL;
            if (num <= VISION_FILTER_MIN && binary_image[y][x] == VISION_WHITE_PIXEL) binary_image[y][x] = VISION_BLACK_PIXEL;
        }
    }
}

static void vision_draw_black_frame(void)
{
    for (uint8 y = 0; y < VISION_IMAGE_H; y++)
    {
        binary_image[y][0] = VISION_BLACK_PIXEL;
        binary_image[y][1] = VISION_BLACK_PIXEL;
        binary_image[y][VISION_IMAGE_W - 1] = VISION_BLACK_PIXEL;
        binary_image[y][VISION_IMAGE_W - 2] = VISION_BLACK_PIXEL;
    }
    for (uint8 x = 0; x < VISION_IMAGE_W; x++)
    {
        binary_image[0][x] = VISION_BLACK_PIXEL;
        binary_image[1][x] = VISION_BLACK_PIXEL;
    }
}
```

- [ ] **Step 3: Add start-point search and eight-neighborhood tracing**

Port `get_start_point()` and `search_l_r()` from the reference as static functions named:

```c
static uint8 vision_get_start_point(uint8 start_row);
static void vision_search_l_r(uint16 break_flag, uint8 *highest);
```

Keep the reference neighbor seed arrays and black-to-white transition checks. Store traced points into `points_l`, `points_r`, `dir_l`, and `dir_r`.

Use only `binary_image`; do not accept an image pointer in `vision_search_l_r()` because this module owns the processing buffer.

- [ ] **Step 4: Add per-row boundary extraction and weighted center**

Implement:

```c
static void vision_get_left(uint16 total_l);
static void vision_get_right(uint16 total_r);
static void vision_compute_center(uint8 highest);
```

`vision_get_left()` and `vision_get_right()` should port the reference behavior:

```c
vision_result.left_boundary[h] = (uint8)(points_l[j][0] + 1);
vision_result.right_boundary[h] = (uint8)(points_r[j][0] - 1);
```

`vision_compute_center()` should calculate:

```c
vision_result.center_line[i] = (uint8)((vision_result.left_boundary[i] + vision_result.right_boundary[i]) >> 1);
vision_result.center_weighted =
    (float)((vision_result.center_line[VISION_IMAGE_H - 2] * 0.07f)
    + (vision_result.center_line[VISION_IMAGE_H - 12] * 0.10f)
    + (vision_result.center_line[VISION_IMAGE_H - 22] * 0.15f)
    + (vision_result.center_line[VISION_IMAGE_H - 27] * 0.25f)
    + (vision_result.center_line[VISION_IMAGE_H - 32] * 0.17f)
    + (vision_result.center_line[VISION_IMAGE_H - 42] * 0.09f)
    + (vision_result.center_line[VISION_IMAGE_H - 52] * 0.07f)
    + (vision_result.center_line[VISION_IMAGE_H - 62] * 0.06f)
    + (vision_result.center_line[VISION_IMAGE_H - 72] * 0.04f));
```

- [ ] **Step 5: Add public process and accessor functions**

Implement:

```c
void vision_image_init(void)
{
    vision_clear_result();
}

void vision_image_process(const uint8 image[VISION_IMAGE_H][VISION_IMAGE_W])
{
    uint8 highest = 0;
    data_stastics_l = 0;
    data_stastics_r = 0;

    vision_copy_image(image);
    vision_turn_to_bin();
    vision_filter();
    vision_draw_black_frame();
    vision_clear_result();
    vision_result.threshold = vision_otsu_threshold(original_image[0], VISION_IMAGE_W, VISION_IMAGE_H);

    if (vision_get_start_point((uint8)(VISION_IMAGE_H - 2)))
    {
        vision_search_l_r(VISION_USE_NUM, &highest);
        vision_get_left(data_stastics_l);
        vision_get_right(data_stastics_r);
        vision_result.highest = highest;
        vision_compute_center(highest);
        vision_result.valid = 1;
    }
}

const VisionImageResult_t *vision_image_get_result(void)
{
    return &vision_result;
}

const uint8 *vision_image_get_original_buffer(void)
{
    return original_image[0];
}

const uint8 *vision_image_get_binary_buffer(void)
{
    return binary_image[0];
}
```

When implementing, avoid recalculating Otsu twice: either preserve `vision_result.threshold` across `vision_clear_result()` or move `vision_clear_result()` before `vision_turn_to_bin()`. The final code should calculate the threshold once per frame.

- [ ] **Step 6: Verify excluded special elements are absent**

Run:

```powershell
rg -n "cross|around|bridge|jump|stop_position|buzzer|target_velocity|small_driver|Encoder_Left|Encoder_Right|pit_all_close" project\code\vision_image.c
```

Expected: no matches except harmless words in comments if any comments were added. Prefer no matches.

### Task 3: Integrate Camera Processing on CM7_1

**Files:**
- Modify: `project/code/app_headfile.h`
- Modify: `project/user/main_cm7_1.c`

**Interfaces:**
- Consumes: `mt9v03x_init`, `mt9v03x_finish_flag`, `mt9v03x_image`, `vision_image_init`, `vision_image_process`.
- Produces: CM7_1 updates the latest vision result whenever a camera frame completes.

- [ ] **Step 1: Include the vision module**

In `project/code/app_headfile.h`, add:

```c
#include "vision_image.h"
```

Place it in the communication/display block near `screen_display.h`.

- [ ] **Step 2: Initialize MT9V03X and vision**

In `project/user/main_cm7_1.c`, after screen initialization and before WiFi initialization, add:

```c
    vision_image_init();
    screen_boot_show_status("Vision", "INIT");
    if (mt9v03x_init())
    {
        screen_boot_show_status("MT9V03X", "ERR");
    }
    else
    {
        screen_boot_show_status("MT9V03X", "OK");
    }
```

The plan intentionally does not stop boot on camera init error, so the existing screen, WiFi, and parameter UI remain usable.

- [ ] **Step 3: Process completed frames in the CM7_1 loop**

In the `while(true)` loop of `project/user/main_cm7_1.c`, before `screen_display_process();`, add:

```c
        if (mt9v03x_finish_flag)
        {
            mt9v03x_finish_flag = 0;
            vision_image_process(mt9v03x_image);
        }
```

- [ ] **Step 4: Verify CM7_0 is untouched**

Run:

```powershell
git diff -- project\user\main_cm7_0.c project\code\control.c project\code\control.h
```

Expected: no diff.

### Task 4: Add Vision UI Page

**Files:**
- Modify: `project/code/screen_display.c`

**Interfaces:**
- Consumes: `vision_image_get_result`, `vision_image_get_original_buffer`.
- Produces: a home-menu-accessible `Vision` page.

- [ ] **Step 1: Add screen enum and text constants**

Add `UI_SCREEN_VISION` before `UI_SCREEN_CONFIRM` in `ui_screen_t`.

Add text constants:

```c
static const uint16_t UI_TEXT_T_TITLE_VISION[] = {0x0056, 0x0069, 0x0073, 0x0069, 0x006F, 0x006E, 0x0000};
static const uint16_t UI_TEXT_T_HOME_VISION[] = {0x0056, 0x0069, 0x0073, 0x0069, 0x006F, 0x006E, 0x0000};
```

- [ ] **Step 2: Add home menu entry**

Add `UI_TEXT_T_HOME_VISION` to `k_home_texts`. Keep `k_home_items` aligned with it if the file still uses both arrays.

Update `ui_handle_home()` index mapping so the new item opens `UI_SCREEN_VISION`. Keep all existing entries reachable.

Recommended order:

```text
Mode
Monitor
Vision
Param
Jump
WiFi
Modules
System
```

- [ ] **Step 3: Add Vision page renderer**

Add:

```c
static void ui_draw_vision(void)
{
    const VisionImageResult_t *result = vision_image_get_result();
    const uint8 *gray = vision_image_get_original_buffer();
    char line[32];
    const uint16 image_x = 0;
    const uint16 image_y = 30;
    const uint16 image_w = VISION_IMAGE_W;
    const uint16 image_h = VISION_IMAGE_H;
    const uint16 ref_x = image_x + VISION_REFERENCE_CENTER;

    if (force_ui_refresh)
    {
        ips200_clear();
        ui_draw_title_text(UI_TEXT_T_TITLE_VISION);
    }

    ips200_show_gray_image(image_x, image_y, gray, VISION_IMAGE_W, VISION_IMAGE_H, image_w, image_h, 0);

    for (uint16 y = 0; y < image_h; y++)
    {
        uint16 screen_y = image_y + y;
        ips200_draw_point(ref_x, screen_y, RGB565_GREEN);

        if (result->valid)
        {
            ips200_draw_point((uint16)(image_x + result->left_boundary[y]), screen_y, RGB565_BLUE);
            ips200_draw_point((uint16)(image_x + result->right_boundary[y]), screen_y, RGB565_RED);
            ips200_draw_point((uint16)(image_x + result->center_line[y]), screen_y, RGB565_YELLOW);
        }
    }

    sprintf(line, "V:%d Th:%03d H:%03d", result->valid, result->threshold, result->highest);
    ips200_show_string(4, 166, line);
    sprintf(line, "C:%06.2f E:%06.2f", result->center_weighted, result->center_weighted - (float)VISION_REFERENCE_CENTER);
    ips200_show_string(4, 188, line);
    ips200_show_string(4, 282, "Blue:L Red:R Yellow:C Green:Ref");
    ui_draw_footer_text_str("BACK Home");
}
```

Use existing RGB565 color names already available through the display driver. If `RGB565_GREEN`, `RGB565_BLUE`, `RGB565_RED`, or `RGB565_YELLOW` are not available in the included headers, replace with the project's existing constants from `zf_common_font.h` or add local `#define` values in `screen_display.h`.

- [ ] **Step 4: Add Vision page key handling**

Add:

```c
static void ui_handle_vision(ui_key_event_t events[UI_KEY_COUNT])
{
    if (events[UI_KEY_BACK])
    {
        ui_set_screen(UI_SCREEN_HOME);
    }
}
```

Update `ui_handle_events()`:

```c
case UI_SCREEN_VISION: ui_handle_vision(events); break;
```

Update `ui_render()`:

```c
case UI_SCREEN_VISION:
    ui_draw_vision();
    break;
```

- [ ] **Step 5: Verify menu mapping**

Run:

```powershell
rg -n "UI_SCREEN_VISION|UI_TEXT_T_HOME_VISION|ui_draw_vision|ui_handle_vision" project\code\screen_display.c
```

Expected: enum, text, draw function, handler, event switch, render switch, and home mapping all appear.

### Task 5: Add IAR CM7_1 Project Entries

**Files:**
- Modify: `project/iar/project_config/cyt4bb7_cm_7_1.ewp`
- Modify: `project/iar/project_config/cyt4bb7_cm_7_1.ewt`

**Interfaces:**
- Consumes: new `project/code/vision_image.c` and `project/code/vision_image.h`.
- Produces: CM7_1 IAR build sees the new module.

- [ ] **Step 1: Add files to CM7_1 app group**

In both project files, add these entries near `screen_display.c` and `screen_display.h`:

```xml
        <file>
            <name>$PROJ_DIR$\..\..\code\vision_image.c</name>
        </file>
        <file>
            <name>$PROJ_DIR$\..\..\code\vision_image.h</name>
        </file>
```

- [ ] **Step 2: Verify CM7_1 includes the files**

Run:

```powershell
rg -n "vision_image\.(c|h)" project\iar\project_config\cyt4bb7_cm_7_1.ewp project\iar\project_config\cyt4bb7_cm_7_1.ewt
```

Expected: four matches, two in `.ewp` and two in `.ewt`.

- [ ] **Step 3: Verify CM7_0 does not include the files**

Run:

```powershell
rg -n "vision_image\.(c|h)" project\iar\project_config\cyt4bb7_cm_7_0.ewp project\iar\project_config\cyt4bb7_cm_7_0.ewt
```

Expected: no matches.

### Task 6: Static Verification and Cleanup

**Files:**
- Inspect: `project/code/vision_image.c`
- Inspect: `project/code/vision_image.h`
- Inspect: `project/code/screen_display.c`
- Inspect: `project/user/main_cm7_1.c`
- Inspect: `project/iar/project_config/cyt4bb7_cm_7_1.ewp`
- Inspect: `project/iar/project_config/cyt4bb7_cm_7_1.ewt`

**Interfaces:**
- Consumes: completed tasks 1-5.
- Produces: evidence that the migration is CM7_1-only and special-element-free.

- [ ] **Step 1: Check for missing declarations or duplicate definitions**

Run:

```powershell
rg -n "vision_image_init|vision_image_process|vision_image_get_result|vision_image_get_original_buffer|vision_image_get_binary_buffer" project\code project\user
```

Expected: each public function has one declaration in `vision_image.h`, one definition in `vision_image.c`, and call sites in CM7_1/UI as appropriate.

- [ ] **Step 2: Check special-element exclusions**

Run:

```powershell
rg -n "cross_fill|around_fill|bridge_fill|cross_stop|jump_judge|SingleBridge|BridgeState|jump_position|stop_position|buzzer|target_velocity|small_driver_set_duty|pit_all_close|Encoder_Left|Encoder_Right" project\code\vision_image.c project\code\vision_image.h
```

Expected: no matches.

- [ ] **Step 3: Check CM7_0 remains untouched**

Run:

```powershell
git diff -- project\user\main_cm7_0.c project\code\control.c project\code\control.h project\iar\project_config\cyt4bb7_cm_7_0.ewp project\iar\project_config\cyt4bb7_cm_7_0.ewt
```

Expected: no diff.

- [ ] **Step 4: Review final diff**

Run:

```powershell
git diff -- project\code\vision_image.h project\code\vision_image.c project\code\app_headfile.h project\code\screen_display.c project\user\main_cm7_1.c project\iar\project_config\cyt4bb7_cm_7_1.ewp project\iar\project_config\cyt4bb7_cm_7_1.ewt
```

Expected: diff contains only the vision module, CM7_1 camera processing, Vision UI page, and CM7_1 IAR project entries.

- [ ] **Step 5: Hardware verification in IAR**

Open `project/iar/project_config/cyt4bb7_cm_7_1.eww` in IAR and build the CM7_1 target. Expected: no compile or link errors related to `vision_image`, `mt9v03x_image`, `mt9v03x_finish_flag`, `ips200_show_gray_image`, or RGB565 color constants.

Flash/run CM7_1 on hardware. Expected:

- Boot continues even if the camera init fails.
- Home menu contains `Vision`.
- `Vision` page shows the grayscale MT9V03X frame.
- Green fixed reference center line is vertically centered at image x = 94.
- Blue/red/yellow overlays appear when `valid` is 1.
- `BACK` returns to home.

## Plan Self-Review

- Spec coverage: all requested parts are covered by tasks 1-6: basic vision processing, center-line logic, grayscale display, overlays, reference line, CM7_1-only integration, and future control interface.
- Placeholder scan: the plan contains no unresolved placeholders.
- Type consistency: public names are consistently `VisionImageResult_t`, `vision_image_*`, `VISION_IMAGE_*`, and `VISION_REFERENCE_CENTER`.
