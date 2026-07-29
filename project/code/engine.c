#include "engine.h"
#include "param.h"

#define ENGINE_NORMAL_LOGICAL_MIN_PWM  (250)
#define ENGINE_NORMAL_LOGICAL_MAX_PWM  (1230)
#define ENGINE_JUMP_LOGICAL_MIN_PWM    (200)
#define ENGINE_JUMP_LOGICAL_MAX_PWM    (1300)

typedef struct
{
    uint32 pwm_channel;
    uint8 reverse;
    int16 trim;
    uint16 logical_min;
    uint16 logical_max;
} servo_channel_cal_t;

// Physical servo mapping: PWM_1=left-front, PWM_2=right-front, PWM_3=right-rear, PWM_4=left-rear.
// Reverse flags follow physical channel direction, not logical left/right side.
// FiveBarLinkageData.c still outputs logical pwm1/pwm2 for each leg.
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

static int engine_limit_pwm(int pwm, int min_pwm, int max_pwm)
{
    if (pwm < min_pwm)
    {
        return min_pwm;
    }
    if (pwm > max_pwm)
    {
        return max_pwm;
    }
    return pwm;
}

static int engine_limit_normal_pwm(int pwm)
{
    return engine_limit_pwm(pwm,
                            ENGINE_NORMAL_LOGICAL_MIN_PWM,
                            ENGINE_NORMAL_LOGICAL_MAX_PWM);
}

static int engine_limit_jump_pwm(int pwm)
{
    return engine_limit_pwm(pwm,
                            ENGINE_JUMP_LOGICAL_MIN_PWM,
                            ENGINE_JUMP_LOGICAL_MAX_PWM);
}

uint32 auu(uint32 c)
{
    if (c > (uint32)ENGINE_NORMAL_LOGICAL_MAX_PWM)
    {
        return (uint32)ENGINE_NORMAL_LOGICAL_MAX_PWM;
    }
    return (uint32)engine_limit_normal_pwm((int)c);
}

static uint32 apply_servo_calibration(int logical_pwm,
                                      const servo_channel_cal_t *cal,
                                      int min_pwm,
                                      int max_pwm)
{
    int pwm = engine_limit_pwm(logical_pwm + cal->trim, min_pwm, max_pwm);

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
    g_pwm_out_1 = apply_servo_calibration(pwm2, &k_left_servo_2,
                                          ENGINE_NORMAL_LOGICAL_MIN_PWM,
                                          ENGINE_NORMAL_LOGICAL_MAX_PWM);
    g_pwm_out_2 = apply_servo_calibration(pwm2, &k_right_servo_2,
                                          ENGINE_NORMAL_LOGICAL_MIN_PWM,
                                          ENGINE_NORMAL_LOGICAL_MAX_PWM);
    g_pwm_out_3 = apply_servo_calibration(pwm1, &k_right_servo_1,
                                          ENGINE_NORMAL_LOGICAL_MIN_PWM,
                                          ENGINE_NORMAL_LOGICAL_MAX_PWM);
    g_pwm_out_4 = apply_servo_calibration(pwm1, &k_left_servo_1,
                                          ENGINE_NORMAL_LOGICAL_MIN_PWM,
                                          ENGINE_NORMAL_LOGICAL_MAX_PWM);
    g_pwm_out_1 = apply_servo_test_override(g_pwm_out_1, 1);
    g_pwm_out_2 = apply_servo_test_override(g_pwm_out_2, 2);
    g_pwm_out_3 = apply_servo_test_override(g_pwm_out_3, 3);
    g_pwm_out_4 = apply_servo_test_override(g_pwm_out_4, 4);

    pwm_init(PWM_1, FREQ, g_pwm_out_1);
    pwm_init(PWM_2, FREQ, g_pwm_out_2);
    pwm_init(PWM_3, FREQ, g_pwm_out_3);
    pwm_init(PWM_4, FREQ, g_pwm_out_4);
}

static void engine_servo_disable_channel(uint8 channel_index)
{
    Cy_Tcpwm_Pwm_Disable((volatile stc_TCPWM_GRP_CNT_t *)&TCPWM0->GRP[0].CNT[channel_index]);
}

void engine_servo_disable(void)
{
    pwm_set_duty(PWM_1, 0);
    pwm_set_duty(PWM_2, 0);
    pwm_set_duty(PWM_3, 0);
    pwm_set_duty(PWM_4, 0);

    engine_servo_disable_channel(13); // PWM_1: left-front
    engine_servo_disable_channel(12); // PWM_2: right-front
    engine_servo_disable_channel(11); // PWM_3: right-rear
    engine_servo_disable_channel(10); // PWM_4: left-rear
}
void engine_servo_enable(void)
{
    pwm_init(PWM_1, FREQ, g_pwm_out_1);
    pwm_init(PWM_2, FREQ, g_pwm_out_2);
    pwm_init(PWM_3, FREQ, g_pwm_out_3);
    pwm_init(PWM_4, FREQ, g_pwm_out_4);
}
void engine_maintain(int pwm1, int pwm2)
{
    engine_left_maintain(pwm1, pwm2);
    engine_right_maintain(pwm1, pwm2);
}

static void engine_left_write_limited(int pwm1, int pwm2,
                                      int min_pwm, int max_pwm)
{
    pwm1 = engine_limit_pwm(pwm1, min_pwm, max_pwm);
    pwm2 = engine_limit_pwm(pwm2, min_pwm, max_pwm);
    g_logic_left_1 = pwm1;
    g_logic_left_2 = pwm2;
    g_pwm_out_1 = apply_servo_calibration(pwm2, &k_left_servo_2,
                                          min_pwm, max_pwm);
    g_pwm_out_4 = apply_servo_calibration(pwm1, &k_left_servo_1,
                                          min_pwm, max_pwm);
    g_pwm_out_1 = apply_servo_test_override(g_pwm_out_1, 1);
    g_pwm_out_4 = apply_servo_test_override(g_pwm_out_4, 4);
    pwm_set_duty(PWM_1, g_pwm_out_1);
    pwm_set_duty(PWM_4, g_pwm_out_4);
}

static void engine_right_write_limited(int pwm1, int pwm2,
                                       int min_pwm, int max_pwm)
{
    pwm1 = engine_limit_pwm(pwm1, min_pwm, max_pwm);
    pwm2 = engine_limit_pwm(pwm2, min_pwm, max_pwm);
    g_logic_right_1 = pwm1;
    g_logic_right_2 = pwm2;
    g_pwm_out_3 = apply_servo_calibration(pwm1, &k_right_servo_1,
                                          min_pwm, max_pwm);
    g_pwm_out_2 = apply_servo_calibration(pwm2, &k_right_servo_2,
                                          min_pwm, max_pwm);
    g_pwm_out_3 = apply_servo_test_override(g_pwm_out_3, 3);
    g_pwm_out_2 = apply_servo_test_override(g_pwm_out_2, 2);
    pwm_set_duty(PWM_3, g_pwm_out_3);
    pwm_set_duty(PWM_2, g_pwm_out_2);
}

void engine_jump_maintain(int pwm1, int pwm2)
{
    pwm1 = engine_limit_jump_pwm(pwm1);
    pwm2 = engine_limit_jump_pwm(pwm2);
    engine_left_write_limited(pwm1, pwm2,
                              ENGINE_JUMP_LOGICAL_MIN_PWM,
                              ENGINE_JUMP_LOGICAL_MAX_PWM);
    engine_right_write_limited(pwm1, pwm2,
                               ENGINE_JUMP_LOGICAL_MIN_PWM,
                               ENGINE_JUMP_LOGICAL_MAX_PWM);
}

void engine_left_maintain(int pwm1, int pwm2)
{
    pwm1 = engine_limit_normal_pwm(pwm1);
    pwm2 = engine_limit_normal_pwm(pwm2);
    engine_left_write_limited(pwm1, pwm2,
                              ENGINE_NORMAL_LOGICAL_MIN_PWM,
                              ENGINE_NORMAL_LOGICAL_MAX_PWM);
}

void engine_right_maintain(int pwm1, int pwm2)
{
    pwm1 = engine_limit_normal_pwm(pwm1);
    pwm2 = engine_limit_normal_pwm(pwm2);
    engine_right_write_limited(pwm1, pwm2,
                               ENGINE_NORMAL_LOGICAL_MIN_PWM,
                               ENGINE_NORMAL_LOGICAL_MAX_PWM);
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
