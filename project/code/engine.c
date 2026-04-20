#include "engine.h"

typedef struct
{
    uint32 pwm_channel;
    uint8 reverse;
    int16 trim;
    uint16 logical_min;
    uint16 logical_max;
} servo_channel_cal_t;

// 实测 4 路物理位置：
// PWM_1 左前，PWM_2 右前，PWM_3 右后，PWM_4 左后
// 实测从中位打到较小占空比时：
// PWM_1/PWM_3 向上，PWM_2/PWM_4 向下
// 因此 reverse 不能按“左边/右边”分，而要按实际通道分。
// FiveBarLinkageData.c 中 servo_control() 输出顺序仍按 pwm1 / pwm2 进入，
// 这里仅负责把每条腿的两个逻辑舵机映射到正确物理通道。
static const servo_channel_cal_t k_left_servo_1 = {PWM_4, 1, 0, 250, 1230};
static const servo_channel_cal_t k_left_servo_2 = {PWM_1, 1, 0, 250, 1230};
static const servo_channel_cal_t k_right_servo_1 = {PWM_3, 0, 0, 250, 1230};
static const servo_channel_cal_t k_right_servo_2 = {PWM_2, 0, 0, 250, 1230};
static uint32 g_pwm_out_1 = 0;
static uint32 g_pwm_out_2 = 0;
static uint32 g_pwm_out_3 = 0;
static uint32 g_pwm_out_4 = 0;
static int g_logic_left_1 = 0;
static int g_logic_left_2 = 0;
static int g_logic_right_1 = 0;
static int g_logic_right_2 = 0;

static uint32 get_servo_test_auto_output(uint8 channel_index)
{
    static uint16 tick = 0;
    static uint8 phase = 0;
    static uint32 outputs[4] = {
        SERVO_TEST_CENTER_DUTY,
        SERVO_TEST_CENTER_DUTY,
        SERVO_TEST_CENTER_DUTY,
        SERVO_TEST_CENTER_DUTY
    };

    if (channel_index == 1)
    {
        uint8 i;

        if (++tick >= SERVO_TEST_STEP_TICKS)
        {
            tick = 0;
            phase = (phase + 1) % 13;
        }

        for (i = 0; i < 4; i++)
        {
            outputs[i] = SERVO_TEST_CENTER_DUTY;
        }

        switch (phase)
        {
            case 1: outputs[0] = SERVO_TEST_MIN_DUTY; break;
            case 2: outputs[0] = SERVO_TEST_MAX_DUTY; break;
            case 4: outputs[1] = SERVO_TEST_MIN_DUTY; break;
            case 5: outputs[1] = SERVO_TEST_MAX_DUTY; break;
            case 7: outputs[2] = SERVO_TEST_MIN_DUTY; break;
            case 8: outputs[2] = SERVO_TEST_MAX_DUTY; break;
            case 10: outputs[3] = SERVO_TEST_MIN_DUTY; break;
            case 11: outputs[3] = SERVO_TEST_MAX_DUTY; break;
            default: break;
        }
    }

    return outputs[channel_index - 1];
}

static uint32 apply_servo_test_override(uint32 pwm_out, uint8 channel_index)
{
#if SERVO_TEST_MODE == 1
    if (channel_index == SERVO_TEST_CHANNEL)
    {
        return (uint32)SERVO_TEST_DUTY;
    }
#elif SERVO_TEST_MODE == 2
    return get_servo_test_auto_output(channel_index);
#endif
    return pwm_out;
}

uint32 buu(uint32 c)
{
    return 1500 - c;
}

uint32 auu(uint32 c)
{
    if (c > 1230)
    {
        return 1230;
    }
    if (c < 250)
    {
        return 250;
    }
    return c;
}

static uint32 apply_servo_calibration(int logical_pwm, const servo_channel_cal_t *cal)
{
    int pwm = logical_pwm + cal->trim;

    if (pwm < (int)cal->logical_min)
    {
        pwm = cal->logical_min;
    }
    if (pwm > (int)cal->logical_max)
    {
        pwm = cal->logical_max;
    }

    if (cal->reverse)
    {
        pwm = (int)buu((uint32)pwm);
    }

    return (uint32)pwm;
}

void engine_init(int pwm1, int pwm2)
{
    pwm1 = (int)auu((uint32)pwm1);
    pwm2 = (int)auu((uint32)pwm2);

    g_logic_left_1 = pwm1;
    g_logic_left_2 = pwm2;
    g_logic_right_1 = pwm1;
    g_logic_right_2 = pwm2;
    g_pwm_out_1 = apply_servo_calibration(pwm2, &k_left_servo_2);
    g_pwm_out_2 = apply_servo_calibration(pwm2, &k_right_servo_2);
    g_pwm_out_3 = apply_servo_calibration(pwm1, &k_right_servo_1);
    g_pwm_out_4 = apply_servo_calibration(pwm1, &k_left_servo_1);
    g_pwm_out_1 = apply_servo_test_override(g_pwm_out_1, 1);
    g_pwm_out_2 = apply_servo_test_override(g_pwm_out_2, 2);
    g_pwm_out_3 = apply_servo_test_override(g_pwm_out_3, 3);
    g_pwm_out_4 = apply_servo_test_override(g_pwm_out_4, 4);

    pwm_init(PWM_1, FREQ, g_pwm_out_1);
    pwm_init(PWM_2, FREQ, g_pwm_out_2);
    pwm_init(PWM_3, FREQ, g_pwm_out_3);
    pwm_init(PWM_4, FREQ, g_pwm_out_4);
}

void engine_left_maintain(int pwm1, int pwm2)
{
    pwm1 = (int)auu((uint32)pwm1);
    pwm2 = (int)auu((uint32)pwm2);

    g_logic_left_1 = pwm1;
    g_logic_left_2 = pwm2;
    g_pwm_out_1 = apply_servo_calibration(pwm2, &k_left_servo_2);
    g_pwm_out_4 = apply_servo_calibration(pwm1, &k_left_servo_1);
    g_pwm_out_1 = apply_servo_test_override(g_pwm_out_1, 1);
    g_pwm_out_4 = apply_servo_test_override(g_pwm_out_4, 4);

    pwm_set_duty(PWM_1, g_pwm_out_1);
    pwm_set_duty(PWM_4, g_pwm_out_4);
}

void engine_right_maintain(int pwm1, int pwm2)
{
    pwm1 = (int)auu((uint32)pwm1);
    pwm2 = (int)auu((uint32)pwm2);

    g_logic_right_1 = pwm1;
    g_logic_right_2 = pwm2;
    g_pwm_out_3 = apply_servo_calibration(pwm1, &k_right_servo_1);
    g_pwm_out_2 = apply_servo_calibration(pwm2, &k_right_servo_2);
    g_pwm_out_3 = apply_servo_test_override(g_pwm_out_3, 3);
    g_pwm_out_2 = apply_servo_test_override(g_pwm_out_2, 2);

    pwm_set_duty(PWM_3, g_pwm_out_3);
    pwm_set_duty(PWM_2, g_pwm_out_2);
}

void engine_jump(void)
{
}

void engine_Stand_change(uint32 left, uint32 right, pid_param_t *pid1, pid_param_t *pid2)
{
    if (left > right)
    {
        auu(left);
        if (left == 750)
        {
            PidChange(pid1, 17000, 0, 1570);
        }
        else if (left > 750 && left < 850)
        {
            PidChange(pid1, 18500, 0, 1570);
            target_motor_Stand = 0;
        }
        else if (left >= 850 && left < 950)
        {
            PidChange(pid1, 20500, 0, 1940);
            PidChange(pid2, 5, 5 / 200, 0);
            target_motor_Stand = 0;
        }
        else
        {
            PidChange(pid1, 24000, 0, 1570);
        }
    }
    else
    {
        auu(right);
        if (left == 750)
        {
            PidChange(pid1, 17000, 0, 1570);
        }
        else if (right > 750 && right < 850)
        {
            PidChange(pid1, 18500, 0, 1570);
            target_motor_Stand = 0;
        }
        else if (right >= 850 && right < 950)
        {
            PidChange(pid1, 20500, 0, 1940);
            PidChange(pid2, 5, 5 / 200, 0);
            target_motor_Stand = 0;
        }
        else
        {
            PidChange(pid1, 22500, 0, 2180);
            PidChange(pid2, 5.3, 5.3 / 200, 0);
            target_motor_Stand = 0;
        }
    }
}
