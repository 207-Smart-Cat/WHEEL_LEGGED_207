#ifndef APP_HEADFILE_H
#define APP_HEADFILE_H

#include "zf_common_headfile.h"

// Temporary navigation hand-push test mode.
// 1 = keep only IMU, motor speed receive, navigation EKF and IPC status upload on Core0.
//     Balance, remote, jump and navigation control task are disabled for nav_x/nav_y testing.
// 0 = normal runtime mode.
#define NAV_HAND_PUSH_TEST_MODE 0   

// Basic utilities
#include "pid.h"
#include "matrix.h"
#include "filter_function.h"

// Parameters, IPC, and runtime status
#include "param.h"
#include "ipc_shared_data.h"
#include "runtime_status.h"
#include "vehicle_supervisor.h"

// Devices and low-level actuators
#include "imu.h"
#include "process_rx.h"
#include "battery_monitor.h"
#include "small_driver_uart_control.h"
#include "camera_assist.h"
#include "camera_test_display.h"
#include "engine.h"
#include "remote.h"

// Control and actions
#include "FiveBarLinkageData.h"
#include "control.h"
#include "jump_control.h"

// Communication and display
#include "wifi.h"
#include "vofa_protocol.h"
#include "screen_display.h"

// Navigation
#include "navigation_data_handling.h"
#include "navigation_tracking.h"
#include "navigation_action.h"

#endif
