/**
 * @file control_task.cpp
 * @author 大帅将军 / lxlx
 * @brief
 * @version 0.3
 * @date 2026-05-28
 *
 * @copyright Copyright (c) 2026
 *
 * @attention
 * @note 
 */

#include "main.h"
#include "control_task.h"
#include "pid_controller.h"
#include "chassis_task.h"
#include "rm_pocket.hpp"
#include "topic_pool.h"
#include "topics.hpp"
#include "bsp_usart.h"
#include "tracking.h"
#include <cmath>
#include <cstdint>
#include "control_Traject.hpp"
#include "Motor.hpp"
#include "point_tracker.hpp"
#include "stair_seq.hpp"   
#include "mission.hpp"

osThreadId_t ControlTaskHandle;
extern volatile float odom_y_mm; 
extern volatile float odom_x_mm;            
extern volatile float lift_2006_dist_mm;
extern volatile bool  lift_at_target;
extern volatile float g_chassis_yaw_deg;
extern volatile float g_yaw_rel_deg; 
extern volatile bool imu_ready;
bool rc_ready = false; 
static bool turn_stick_last = false; 
constexpr int32_t kTurnStickThresh = 400;    
constexpr float   kTurn90  = 90.0f;
constexpr float   kTurn180 = 180.0f;



// ===== 发布者 =====
TypedTopicPublisher<pub_chassis_cmd> chassis_data_pub("chassis_cmd");
pub_chassis_cmd rm_cmd{};

// static TypedTopicPublisher<QR_code_cmd_t> qr_code_cmd_pub("qr_code_cmd");   
// static QR_code_cmd_t qr_code_cmd{};

TypedTopicSubscriber<pub_RM_Data> control_rm_sub("rm", 8);
pub_RM_Data control_rm_cmd{};
pub_RM_Data control_rm_cmd_last{};

TypedTopicPublisher<pub_lift_cmd> lift_data_pub("lift_cmd");
pub_lift_cmd lift_cmd{};

TypedTopicSubscriber<pub_Position_Data> control_position_sub("position", 8);
pub_Position_Data control_position_msg{};
pub_Position_Data control_position{};

pub_chassis_cmd robot_v_aim_cmd{};
pub_chassis_cmd state_now_cmd{};

PointTracker tracker{};//点跟踪器对象
pub_chassis_cmd track_out{}; 

void controlInit() {
    if (!chassis_data_pub.IsValid()) {
        return;
    }

    if (!control_rm_sub.IsValid()) {
        return;
    }

    if (!control_position_sub.IsValid()) {
        return;
    }
    control_rm_cmd.joyLHori = kJoyCenter;
    control_rm_cmd.joyLVert = kJoyCenter;
    control_rm_cmd.joyRHori = kJoyCenter;
    control_rm_cmd.joyRVert = kJoyCenter;
    control_rm_cmd_last = control_rm_cmd;
    
    // if (!control_ir_sub.IsValid()) {
    //     return;
    // }
    // if(!qr_code_cmd_pub.IsValid()) {
    //     return;
    // }
    // if(!qr_code_data_sub.IsValid()) {
    //     return;
    // }
}


float rm_angle_deg;
float v_aim;

void controlTask(void *argument) {
    TickType_t currentTime = xTaskGetTickCount();
    TickType_t rcUpdateTime = currentTime;
    constexpr TickType_t kRemoteInputTimeout = pdMS_TO_TICKS(500);


    controlInit();
    // uint32_t last_time = HAL_GetTick();

    for (;;) {
        // static float controlTask_dt = 0.0f;
        // static uint32_t controlTask_DWT_CNT = 0;
        // controlTask_dt = DWT_GetDeltaT(&controlTask_DWT_CNT);
        //test begin
        const TickType_t now = xTaskGetTickCount();
        const bool rm_updated = control_rm_sub.TryGet(&control_rm_cmd);
        if (rm_updated) {
            rcUpdateTime = now;
            rc_ready = true;  
        } else if (now - rcUpdateTime > kRemoteInputTimeout) {
            control_rm_cmd.joyLHori = kJoyCenter;
            control_rm_cmd.joyLVert = kJoyCenter;
            control_rm_cmd.joyRHori = kJoyCenter;
            control_rm_cmd.joyRVert = kJoyCenter;
        }
        control_rm_cmd_last = control_rm_cmd;
        if (!rc_ready)
        {
            vTaskDelayUntil(&currentTime, 5);
            continue;
        }

        const bool tr_mode = (control_rm_cmd.swC == RM_3_POS_SW_State_t::MIDDLE);
        static bool swC_last = false;
        if (tr_mode && !swC_last && imu_ready)
        {
            tracker.SetTarget(odom_x_mm, odom_y_mm, g_yaw_rel_deg); 
            tracker.SetTarget(-1600.0f, 2095.0f, 0.0f); 
        }
        swC_last = tr_mode;

        lift_cmd.high_mode = (control_rm_cmd.swD == RM_2_POS_SW_State_t::DOWN);
        lift_cmd.lift_2006_input = JoyToVelocity(control_rm_cmd.joyRVert, kJoyDeadZoneRight, 1.0f);

        const bool turn_mode = (control_rm_cmd.swB == RM_3_POS_SW_State_t::MIDDLE ||
                                control_rm_cmd.swB == RM_3_POS_SW_State_t::DOWN);
        const float turn_angle =
            (control_rm_cmd.swB == RM_3_POS_SW_State_t::DOWN) ? kTurn180 : kTurn90;

        const int32_t rstick = (int32_t)control_rm_cmd.joyRHori - (int32_t)kJoyCenter;
        const bool stick_left  = (rstick < -kTurnStickThresh);
        const bool stick_right = (rstick >  kTurnStickThresh);
        {
            rm_cmd.linear_x_ = JoyToVelocity(control_rm_cmd.joyLHori, kJoyDeadZoneLeft, MAX_VELOCITY_LINEAR);
            rm_cmd.linear_y_ = JoyToVelocity(control_rm_cmd.joyLVert, kJoyDeadZoneLeft, MAX_VELOCITY_LINEAR);
            rm_cmd.omega_    = -JoyToVelocity(control_rm_cmd.joyRHori, kJoyDeadZoneRight, MAX_VELOCITY_ANGULAR);

            rm_angle_deg = atan2(rm_cmd.linear_y_, rm_cmd.linear_x_) / kDegToRad;
            v_aim = sqrt(rm_cmd.linear_x_ * rm_cmd.linear_x_ + rm_cmd.linear_y_ * rm_cmd.linear_y_);
            robot_v_aim_cmd.linear_x_ = v_aim * cos((rm_angle_deg - state_now_cmd.omega_) * kDegToRad);
            robot_v_aim_cmd.linear_y_ = v_aim * sin((rm_angle_deg - state_now_cmd.omega_) * kDegToRad);
            robot_v_aim_cmd.omega_ = turn_mode ? 0.0f : rm_cmd.omega_;
            const bool stick_push = stick_left || stick_right;
            robot_v_aim_cmd.turn_delta_deg = 0.0f;                      
            if (turn_mode && !lift_cmd.high_mode && stick_push && !turn_stick_last) { 
                robot_v_aim_cmd.turn_delta_deg = stick_left ? +turn_angle : -turn_angle;
            }
            if (turn_mode)
            {
                robot_v_aim_cmd.linear_x_ = 0.0f;
                robot_v_aim_cmd.linear_y_ = 0.0f;
            }
            turn_stick_last = (turn_mode && stick_push);
        }
        robot_v_aim_cmd.yaw_hold_enable = !lift_cmd.high_mode;   
        if (lift_cmd.high_mode) {
            robot_v_aim_cmd.linear_x_ = 0.0f;      //高位模式底盘不动
            robot_v_aim_cmd.linear_y_ = 0.0f;
            robot_v_aim_cmd.omega_    = 0.0f;
        }
        track_out = tracker.Update(odom_x_mm,  odom_y_mm,  g_yaw_rel_deg);
        if (tr_mode && !lift_cmd.high_mode)
        {
            float vx = track_out.linear_x_;
            float vy = track_out.linear_y_;
            const float vmax = 0.3f;
            const float vmag = sqrtf(vx * vx + vy * vy);
            if (vmag > vmax) {
                const float k = vmax / vmag;
                vx *= k;
                vy *= k;
            }
            robot_v_aim_cmd.linear_x_ = vx;
            robot_v_aim_cmd.linear_y_ = vy;
            robot_v_aim_cmd.omega_ = 0.0f;
            robot_v_aim_cmd.turn_delta_deg = 0.0f;
        }
        // swA：高位/低位切换
        lift_cmd.request_high = false;
        lift_cmd.request_low = false;
        if (rm_updated && (control_rm_cmd.swA != control_rm_cmd.swA_last)) {
            if (control_rm_cmd.swA == RM_2_POS_SW_State_t::UP) {
                lift_cmd.request_low = true;
            } else {
                lift_cmd.request_high = true;
            }
        }
        mission_Update(rm_updated, control_rm_cmd,
                       odom_x_mm, odom_y_mm, g_yaw_rel_deg,
                       tracker, track_out, robot_v_aim_cmd);
        stairSeq_Update(rm_updated, control_rm_cmd, odom_x_mm, odom_y_mm, g_yaw_rel_deg,
                        lift_2006_dist_mm,
                        lift_at_target, lift_cmd, robot_v_aim_cmd, tracker, track_out);

        chassis_data_pub.Publish(robot_v_aim_cmd);
        lift_data_pub.Publish(lift_cmd);
        vTaskDelayUntil(&currentTime, 5);
    }
}