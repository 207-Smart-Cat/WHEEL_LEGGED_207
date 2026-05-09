#include "vehicle_supervisor.h"

#include "engine.h"
#include "jump_control.h"
#include "runtime_status.h"
#include "small_driver_uart_control.h"

static volatile uint8 g_vehicle_emergency_stop = 0;
static volatile vehicle_event_source_t g_vehicle_emergency_source = VEHICLE_EVENT_SOURCE_NONE;

/*
  函数名称：Vehicle_Emergency_Stop
  变量：source：触发紧急制动的来源
  返回：无
  作用：统一执行整车紧急制动，关闭电机输出、平衡模块、舵机模块、跳跃状态机和四路舵机 PWM。
*/
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

/*
  函数名称：Vehicle_Is_Emergency_Stop
  变量：无
  返回：1 表示紧急制动已触发，0 表示未触发
  作用：给控制环和中断判断当前是否处于紧急制动状态，防止后续控制代码重新写入输出。
*/
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

/*
  函数名称：Vehicle_Get_Emergency_Source
  变量：无
  返回：最近一次紧急制动来源
  作用：后续调试或屏幕显示时可以查看紧急制动由 WiFi、遥控或屏幕按键触发。
*/
vehicle_event_source_t Vehicle_Get_Emergency_Source(void)
{
    return g_vehicle_emergency_source;
}
