#ifndef CAMERA_TEST_DISPLAY_H
#define CAMERA_TEST_DISPLAY_H

#include "zf_common_typedef.h"

void CameraTestDisplay_ResetRenderState(void);
void CameraTestDisplay_Render(void);
void CameraTestDisplay_AdjustExposure(int16 delta);
void CameraTestDisplay_DrawLaneBoundaries(void);
void CameraTestDisplay_DrawCenterLines(void);
void CameraTestDisplay_DrawStatusText(void);

#endif
