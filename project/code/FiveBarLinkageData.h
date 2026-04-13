#ifndef FIVE_BAR_LINKAGE_DATA_H
#define FIVE_BAR_LINKAGE_DATA_H

typedef enum
{
    SERVO_LEG_LEFT = 0,
    SERVO_LEG_RIGHT = 1
} servo_leg_id_t;

int servo_target_valid(servo_leg_id_t leg_id, float x, float y);
void servo_control(servo_leg_id_t leg_id, float x, float y, int *leg1, int *leg2);

#endif // FIVE_BAR_LINKAGE_DATA_H
