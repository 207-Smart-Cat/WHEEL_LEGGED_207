#include <stdio.h>

#include "camera_assist.h"

static int expect_align_waits_until_ok_release(void)
{
    CameraAlignLogic_t logic;

    CameraAlignLogic_Reset(&logic);

    if (CameraAlignLogic_Update(&logic, 1U) != 0U)
    {
        printf("align completed before key press\n");
        return 0;
    }
    if (CameraAlignLogic_Update(&logic, 0U) != 0U)
    {
        printf("align completed while key still pressed\n");
        return 0;
    }
    if (CameraAlignLogic_Update(&logic, 0U) != 0U)
    {
        printf("align completed on repeated pressed sample\n");
        return 0;
    }
    if (CameraAlignLogic_Update(&logic, 1U) != 1U)
    {
        printf("align did not complete on ok release\n");
        return 0;
    }

    return 1;
}

static int expect_logic_rearms_after_reset(void)
{
    CameraAlignLogic_t logic;

    CameraAlignLogic_Reset(&logic);
    (void)CameraAlignLogic_Update(&logic, 0U);
    (void)CameraAlignLogic_Update(&logic, 1U);

    CameraAlignLogic_Reset(&logic);
    if (CameraAlignLogic_Update(&logic, 1U) != 0U)
    {
        printf("align completed immediately after reset\n");
        return 0;
    }

    return 1;
}

int main(void)
{
    if (!expect_align_waits_until_ok_release())
    {
        return 1;
    }

    if (!expect_logic_rearms_after_reset())
    {
        return 1;
    }

    return 0;
}
