/**
 * @file chassis_task.cpp
 * @author 大帅将军
 * @brief 底盘任务实现
 * @version 0.1
 * @date 2026-04-21
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */

#include "chassis_task.h"
#include "Motor.hpp"
#include "chassis_solution.hpp"
#include "com_config.h"
#include "pid_controller.h"
#include "topic_pool.h"
#include "topics.hpp"
#include "tool.hpp"
#include <array>

// 任务句柄
osThreadId_t ChassisTaskHandle;

// 底盘电机实例声明
extern C620Motor chassis_motor1, chassis_motor2, chassis_motor3, chassis_motor4;

//里程
volatile float odom_x_mm = 0.0f;                     // x 里程(mm)
volatile float odom_y_mm = 0.0f;                     // y 里程(mm)
static float odom_last_deg[4] = {0.0f, 0.0f, 0.0f, 0.0f};  
static bool  odom_ready = false;                          
float ds[4];  

volatile float yaw_lock_deg   = 0.0f;            // 锁头目标角
static bool  yaw_lock_valid = false;           // 目标角捕获标志
volatile float g_chassis_yaw_deg = 0.0f;
volatile bool  imu_ready = false;   
volatile float yaw_base_deg = 0.0f;
volatile float g_yaw_rel_deg = 0.0f; 

constexpr float kOdomWheelScale = 0.9634f;// 里程计轮子缩放系数
// IMU数据订阅
static TypedTopicSubscriber<pub_imu_data> imu_sub("imu_data", 8);
pub_imu_data imu_msg_recv{};

// 底盘控制命令订阅
static TypedTopicSubscriber<pub_chassis_cmd> chassis_cmd_sub("chassis_cmd", 8);
pub_chassis_cmd chassis_chassis_cmd{};

//将实例移出命名空间以在debug处访问
OmniChassis Omnichassis_solver(chassis_motor1, chassis_motor2, chassis_motor3,
                chassis_motor4);
namespace {//omni底盘解算器实例
// OmniChassis Omnichassis_solver(chassis_motor1, chassis_motor2, chassis_motor3,
//                 chassis_motor4);
// 每个轮子的PID参数配置
const std::array<OmniChassis::SpeedPidParam, OmniChassis::kWheelCount>
    kOmniWheelSpeedPidParams = {
        // OmniChassis::SpeedPidParam(105.0f, 75000.0f, 0.002f, 16000.0f, 0.5f, NONE), // 左上
        // OmniChassis::SpeedPidParam(105.0f, 75000.0f, 0.002f, 16000.0f, 0.5f, NONE), // 右上
        // OmniChassis::SpeedPidParam(105.0f, 75000.0f, 0.002f, 16000.0f, 0.5f, NONE), // 左下
        // OmniChassis::SpeedPidParam(105.0f, 75000.0f, 0.002f, 16000.0f, 0.5f, NONE), // 右下
        // OmniChassis::SpeedPidParam(1200.0f, 750.0f, 0.0f, 16000.0f, 0.5f, NONE), // 左上
        // OmniChassis::SpeedPidParam(1200.0f, 750.0f, 0.0f, 16000.0f, 0.5f, NONE), // 右上
        // OmniChassis::SpeedPidParam(1200.0f, 750.0f, 0.0f, 16000.0f, 0.5f, NONE), // 左下
        // OmniChassis::SpeedPidParam(1200.0f, 750.0f, 0.0f, 16000.0f, 0.5f, NONE), // 右下
        OmniChassis::SpeedPidParam(3500.0f, 7600.0f, 0.0f, 16000.0f, 0.0f, IMCREATEMENT_OF_OUT), // 左上
        OmniChassis::SpeedPidParam(3500.0f, 7600.0f, 0.0f, 16000.0f, 0.0f, IMCREATEMENT_OF_OUT), // 右上
        OmniChassis::SpeedPidParam(3500.0f, 7600.0f, 0.0f, 16000.0f, 0.0f, IMCREATEMENT_OF_OUT), // 左下
        OmniChassis::SpeedPidParam(3500.0f, 7600.0f, 0.0f, 16000.0f, 0.0f, IMCREATEMENT_OF_OUT), // 右下
    };

const std::array<OmniChassis::SpeedPidParam, OmniChassis::kWheelCount>
    kOmniWheelAnglePidParams = {
        OmniChassis::SpeedPidParam(1200.0f, 750.0f, 0.0f, 16000.0f, 0.5f, NONE), // 左上
        OmniChassis::SpeedPidParam(1200.0f, 750.0f, 0.0f, 16000.0f, 0.5f, NONE), // 右上
        OmniChassis::SpeedPidParam(1200.0f, 750.0f, 0.0f, 16000.0f, 0.5f, NONE), // 左下
        OmniChassis::SpeedPidParam(1200.0f, 750.0f, 0.0f, 16000.0f, 0.5f, NONE), // 右下
    };
}

PID_t yaw_hold_pid = {
    .Kp = 5.7f,                               
    .Ki = 0.1f,
    .Kd = 0.0f,
    .MaxOut = 2.5f,                           // 底盘锁头
    .IntegralLimit = 0.5f,
    .DeadBand = 0.3f * DEGREE_2_RAD,           
    .Improve = Integral_Limit,
};
static void odom_Update(void)
{
    const float now_deg[4] = {
        chassis_motor1.getCurrentSumPos(),
        chassis_motor2.getCurrentSumPos(),
        chassis_motor3.getCurrentSumPos(),
        chassis_motor4.getCurrentSumPos(),
    };
     if (!odom_ready) {
        odom_last_deg[0] = now_deg[0];
        odom_last_deg[1] = now_deg[1];
        odom_last_deg[2] = now_deg[2];
        odom_last_deg[3] = now_deg[3];
        odom_ready = true;
        return;
    }
                                   
    for (int i = 0; i < 4; i++) {
        ds[i] = (now_deg[i] - odom_last_deg[i]) * DEGREE_2_RAD * (0.15378f / 2.0f) * kOdomWheelScale * (-1.0f );
        odom_last_deg[i] = now_deg[i];
    }
    const float kInv2Sqrt2 = 1.0f / (2.0f * 1.41421356f);   
    const float dx = ((ds[2] + ds[3]) - (ds[0] + ds[1])) * kInv2Sqrt2;
    const float dy = ((ds[1] + ds[3]) - (ds[0] + ds[2])) * kInv2Sqrt2;
    float yaw_rad = g_yaw_rel_deg * DEGREE_2_RAD;
    float s = sinf(yaw_rad);
    float c = cosf(yaw_rad);
    float wdx = dx * c - dy * s;     
    float wdy = dx * s + dy * c;
    odom_x_mm += wdx * 1000.0f;
    odom_y_mm += wdy * 1000.0f;// 米 → mm

}

static inline void chassisInit() {
    Omnichassis_solver.configureSpeedPid(kOmniWheelSpeedPidParams);
    Omnichassis_solver.configureAnglePid(kOmniWheelAnglePidParams);
    if (!chassis_cmd_sub.IsValid()) {
        return;
    }
}

void chassisTask(void *argument) {
    TickType_t currentTime = xTaskGetTickCount();

    chassisInit();

    for (;;) {
        if (imu_sub.TryGet(&imu_msg_recv))
        {
            g_chassis_yaw_deg = imu_msg_recv.yaw_rad * RAD_2_DEGREE; 
            static bool base_yaw_valid = false;
            if (!base_yaw_valid) {
                yaw_lock_deg = g_chassis_yaw_deg;
                yaw_base_deg = g_chassis_yaw_deg;
                base_yaw_valid = true; }
            g_yaw_rel_deg = normalizeDeg(g_chassis_yaw_deg - yaw_base_deg); 
            imu_ready = true; 
        }
        odom_Update(); 
        if (chassis_cmd_sub.TryGet(&chassis_chassis_cmd))
        {
            if (chassis_chassis_cmd.turn_delta_deg != 0.0f) {
                yaw_lock_deg = normalizeDeg(yaw_lock_deg + chassis_chassis_cmd.turn_delta_deg);
            }
        }// 无论有没有新命令，都持续执行解算
        pub_chassis_cmd final_cmd = chassis_chassis_cmd;
         if (chassis_chassis_cmd.yaw_hold_enable && imu_ready) {
            if (!yaw_lock_valid) {                      
                yaw_lock_valid = true;
                PID_Init(&yaw_hold_pid);
            }
                const bool operator_rotating =
                    fabsf(chassis_chassis_cmd.omega_) > 0.05f;
                if (operator_rotating)
                {
                    yaw_lock_deg = g_chassis_yaw_deg;
                    yaw_hold_pid.Iout = 0.0f;
                    yaw_hold_pid.ITerm = 0.0f;
                }
                else
                {
                    const float err_rad =
                        normalizeDeg(yaw_lock_deg - g_chassis_yaw_deg) * DEGREE_2_RAD;
                    final_cmd.omega_ = PID_Calculate(&yaw_hold_pid, 0.0f, err_rad);
                }
            }
            else
            {
                yaw_lock_valid = false; // 退出锁头 
            }
        Omnichassis_solver.run(final_cmd);
        vTaskDelayUntil(&currentTime, 1);
    }
}