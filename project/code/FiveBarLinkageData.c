#include <math.h>
#include <stdio.h>
#include "FiveBarLinkageData.h"
#include "param.h"

typedef struct
{
    int leg1_pwm_last;
    int leg2_pwm_last;
    float phi1_last_deg;
    float phi4_last_signed_deg;
} servo_leg_state_t;

static servo_leg_state_t g_servo_leg_state[2] = {
    {250, 750, 180.0f, 0.0f},
    {250, 750, 180.0f, 0.0f}
};

static float normalize_360(float angle_deg)
{
    while (angle_deg >= 360.0f)
    {
        angle_deg -= 360.0f;
    }
    while (angle_deg < 0.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static float normalize_signed(float angle_deg)
{
    angle_deg = normalize_360(angle_deg);
    if (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }
    return angle_deg;
}

static float abs_local(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float circular_distance_deg(float a_deg, float b_deg)
{
    return abs_local(normalize_signed(a_deg - b_deg));
}

static int solve_phi1_candidates(float x_target, float y_target, float *phi1_a_deg, float *phi1_b_deg)
{
    float x_plus = x_target + L5 / 2.0f;
    float a = 2.0f * x_plus * L1;
    float b = 2.0f * y_target * L1;
    float c = (x_plus * x_plus) + (y_target * y_target) + (L1 * L1) - (L2 * L2);
    float denom = (a * a) + (b * b);
    float radicand = denom - (c * c);

    if (radicand < 0.0f || denom <= 0.0f)
    {
        return 0;
    }

    {
        float psi = atan2f(b, a);
        float alpha = acosf(c / sqrtf(denom));
        *phi1_a_deg = normalize_360((psi + alpha) * (180.0f / PI));
        *phi1_b_deg = normalize_360((psi - alpha) * (180.0f / PI));
    }

    return 1;
}

static int solve_phi4_candidates(float x_target, float y_target, float *phi4_a_signed_deg, float *phi4_b_signed_deg)
{
    float x_minus = x_target - L5 / 2.0f;
    float a = 2.0f * x_minus * L4;
    float b = 2.0f * y_target * L4;
    float c = (x_minus * x_minus) + (y_target * y_target) + (L4 * L4) - (L3 * L3);
    float denom = (a * a) + (b * b);
    float radicand = denom - (c * c);

    if (radicand < 0.0f || denom <= 0.0f)
    {
        return 0;
    }

    {
        float psi = atan2f(b, a);
        float alpha = acosf(c / sqrtf(denom));
        *phi4_a_signed_deg = normalize_signed((psi - alpha) * (180.0f / PI));
        *phi4_b_signed_deg = normalize_signed((psi + alpha) * (180.0f / PI));
    }

    return 1;
}

static int phi1_candidate_valid(float phi1_deg)
{
    return (phi1_deg >= 99.0f && phi1_deg <= 261.0f);
}

static int phi4_candidate_valid(float phi4_signed_deg)
{
    return (phi4_signed_deg >= -81.0f && phi4_signed_deg <= 81.0f);
}

static int select_phi1_deg(float candidate_a_deg, float candidate_b_deg, float last_deg, float *selected_deg)
{
    int valid_a = phi1_candidate_valid(candidate_a_deg);
    int valid_b = phi1_candidate_valid(candidate_b_deg);

    if (!valid_a && !valid_b)
    {
        return 0;
    }
    if (valid_a && !valid_b)
    {
        *selected_deg = candidate_a_deg;
        return 1;
    }
    if (!valid_a && valid_b)
    {
        *selected_deg = candidate_b_deg;
        return 1;
    }

    {
        float dist_a = circular_distance_deg(candidate_a_deg, last_deg);
        float dist_b = circular_distance_deg(candidate_b_deg, last_deg);
        *selected_deg = (dist_a <= dist_b) ? candidate_a_deg : candidate_b_deg;
    }
    return 1;
}

static int select_phi4_signed_deg(float candidate_a_signed_deg, float candidate_b_signed_deg, float last_signed_deg, float *selected_signed_deg)
{
    int valid_a = phi4_candidate_valid(candidate_a_signed_deg);
    int valid_b = phi4_candidate_valid(candidate_b_signed_deg);

    if (!valid_a && !valid_b)
    {
        return 0;
    }
    if (valid_a && !valid_b)
    {
        *selected_signed_deg = candidate_a_signed_deg;
        return 1;
    }
    if (!valid_a && valid_b)
    {
        *selected_signed_deg = candidate_b_signed_deg;
        return 1;
    }

    {
        float dist_a = abs_local(candidate_a_signed_deg - last_signed_deg);
        float dist_b = abs_local(candidate_b_signed_deg - last_signed_deg);
        *selected_signed_deg = (dist_a <= dist_b) ? candidate_a_signed_deg : candidate_b_signed_deg;
    }
    return 1;
}

static int phi1_deg_to_pwm(float phi1_deg)
{
    return (int)(((phi1_deg - 90.0f) / 180.0f) * 1000.0f + 250.0f);
}

static int phi4_signed_deg_to_pwm(float phi4_signed_deg)
{
    return (int)(((phi4_signed_deg + 90.0f) / 180.0f) * 1000.0f + 250.0f);
}

int servo_target_valid(servo_leg_id_t leg_id, float x, float y)
{
    float phi1_a_deg;
    float phi1_b_deg;
    float phi4_a_signed_deg;
    float phi4_b_signed_deg;
    (void)leg_id;

    if (!solve_phi1_candidates(x, y, &phi1_a_deg, &phi1_b_deg) ||
        !solve_phi4_candidates(x, y, &phi4_a_signed_deg, &phi4_b_signed_deg))
    {
        return 0;
    }

    return ((phi1_candidate_valid(phi1_a_deg) || phi1_candidate_valid(phi1_b_deg)) &&
            (phi4_candidate_valid(phi4_a_signed_deg) || phi4_candidate_valid(phi4_b_signed_deg)));
}

void servo_control(servo_leg_id_t leg_id, float x, float y, int *leg1, int *leg2)
{
    float phi1_a_deg;
    float phi1_b_deg;
    float phi4_a_signed_deg;
    float phi4_b_signed_deg;
    float phi1_selected_deg;
    float phi4_selected_signed_deg;
    servo_leg_state_t *state;

    if (leg1 == NULL || leg2 == NULL)
    {
        return;
    }

    if (leg_id != SERVO_LEG_LEFT && leg_id != SERVO_LEG_RIGHT)
    {
        *leg1 = 250;
        *leg2 = 750;
        return;
    }

    state = &g_servo_leg_state[(int)leg_id];

    if (!solve_phi1_candidates(x, y, &phi1_a_deg, &phi1_b_deg) ||
        !solve_phi4_candidates(x, y, &phi4_a_signed_deg, &phi4_b_signed_deg))
    {
        *leg1 = state->leg1_pwm_last;
        *leg2 = state->leg2_pwm_last;
        return;
    }

    if (!select_phi1_deg(phi1_a_deg, phi1_b_deg, state->phi1_last_deg, &phi1_selected_deg) ||
        !select_phi4_signed_deg(phi4_a_signed_deg, phi4_b_signed_deg, state->phi4_last_signed_deg, &phi4_selected_signed_deg))
    {
        *leg1 = state->leg1_pwm_last;
        *leg2 = state->leg2_pwm_last;
        return;
    }

    *leg1 = phi1_deg_to_pwm(phi1_selected_deg);
    *leg2 = phi4_signed_deg_to_pwm(phi4_selected_signed_deg);

    state->leg1_pwm_last = *leg1;
    state->leg2_pwm_last = *leg2;
    state->phi1_last_deg = phi1_selected_deg;
    state->phi4_last_signed_deg = phi4_selected_signed_deg;
}
