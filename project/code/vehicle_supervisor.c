#include "vehicle_supervisor.h"

#include "engine.h"
#include "jump_control.h"
#include "runtime_status.h"
#include "small_driver_uart_control.h"

static volatile uint8 g_vehicle_emergency_stop = 0;
static volatile vehicle_event_source_t g_vehicle_emergency_source = VEHICLE_EVENT_SOURCE_NONE;

/* Vehicle_Emergency_Stop: disable motor, balance, servo and jump outputs immediately. */
void Vehicle_Emergency_Stop(vehicle_event_source_t source)
{
    g_vehicle_emergency_stop = 1;
    g_vehicle_emergency_source = source;

    Runtime_Set_Module_Enabled(RUNTIME_MODULE_MOTOR, 0);
    Runtime_Set_Module_Enabled(RUNTIME_MODULE_BALANCE, 0);
    Runtime_Set_Module_Enabled(RUNTIME_MODULE_SERVO, 0);
    Runtime_Set_Motor_Reason(RUNTIME_REASON_MOTOR_OFF);
    Runtime_Set_Balance_Reason(RUNTIME_REASON_BALANCE_OFF);
    Runtime_Set_Servo_Reason(RUNTIME_REASON_SERVO_OFF);

    small_driver_set_duty(0, 0);
    jump_force_idle();
    engine_servo_disable();
}

void Vehicle_Emergency_Recover(vehicle_event_source_t source)
{
    g_vehicle_emergency_stop = 0;
    g_vehicle_emergency_source = source;

    Runtime_Set_Module_Enabled(RUNTIME_MODULE_MOTOR, 1);
    Runtime_Set_Module_Enabled(RUNTIME_MODULE_BALANCE, 1);
    Runtime_Set_Module_Enabled(RUNTIME_MODULE_SERVO, 1);
    Runtime_Set_Motor_Reason(RUNTIME_REASON_NORMAL);
    Runtime_Set_Balance_Reason(RUNTIME_REASON_NORMAL);
    Runtime_Set_Servo_Reason(RUNTIME_REASON_NORMAL);
    engine_servo_enable();

    small_driver_set_duty(0, 0);
    small_driver_request_startup_ramp_reset();
    jump_force_idle();
}
/* Vehicle_Is_Emergency_Stop: return whether emergency stop is active. */
uint8 Vehicle_Is_Emergency_Stop(void)
{
    if (g_vehicle_emergency_stop)
    {
        return 1;
    }

    if (!Runtime_Is_Module_Enabled(RUNTIME_MODULE_MOTOR) &&
        !Runtime_Is_Module_Enabled(RUNTIME_MODULE_BALANCE) &&
        !Runtime_Is_Module_Enabled(RUNTIME_MODULE_SERVO))
    {
        return 1;
    }

    return 0;
}

/* Vehicle_Get_Emergency_Source: return the latest emergency event source. */
vehicle_event_source_t Vehicle_Get_Emergency_Source(void)
{
    return g_vehicle_emergency_source;
}
