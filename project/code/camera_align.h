#ifndef CAMERA_ALIGN_H
#define CAMERA_ALIGN_H

#include "zf_common_typedef.h"

typedef struct
{
    uint8 ok_seen_down;
} CameraAlignLogic_t;

static inline void CameraAlignLogic_Reset(CameraAlignLogic_t *logic)
{
    logic->ok_seen_down = 0U;
}

static inline uint8 CameraAlignLogic_Update(CameraAlignLogic_t *logic, uint8 ok_level)
{
    if (ok_level == 0U)
    {
        logic->ok_seen_down = 1U;
        return 0U;
    }

    if (logic->ok_seen_down)
    {
        logic->ok_seen_down = 0U;
        return 1U;
    }

    return 0U;
}

uint8 camera_align(void);
void camera_align_reset(void);

#endif
