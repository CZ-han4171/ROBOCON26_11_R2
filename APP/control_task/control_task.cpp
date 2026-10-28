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

#include <cstdint>
#include <atomic>

#include "control_Traject.hpp"
#include "Motor.hpp"

osThreadId_t ControlTaskHandle;

static std::atomic<uint32_t> arm_release_count{0U};

void requestArmReleaseKFS() {
    arm_release_count.fetch_add(1U, std::memory_order_relaxed);
}

// ===== 发布者 =====
TypedTopicPublisher<pub_chassis_cmd> chassis_data_pub("chassis_cmd");
pub_chassis_cmd rm_cmd{};

// static TypedTopicPublisher<QR_code_cmd_t> qr_code_cmd_pub("qr_code_cmd");   
// static QR_code_cmd_t qr_code_cmd{};

TypedTopicSubscriber<pub_RM_Data> control_rm_sub("rm", 8);
pub_RM_Data control_rm_cmd{};

TypedTopicSubscriber<pub_Position_Data> control_position_sub("position", 8);
pub_Position_Data control_position_msg{};
pub_Position_Data control_position{};

pub_chassis_cmd robot_v_aim_cmd{};
pub_chassis_cmd state_now_cmd{};

extern VESCMotor U8_3;
int32_t U8_3_erpm = 0;

TypedTopicPublisher<arm_sequence_cmd> arm_sequence_cmd_pub("arm_sequence_cmd");



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
    // if (!control_ir_sub.IsValid()) {
    //     return;
    // }
    // if(!qr_code_cmd_pub.IsValid()) {
    //     return;
    // }
    // if(!qr_code_data_sub.IsValid()) {
    //     return;
    // }
    if(!arm_sequence_cmd_pub.IsValid()){
        return;
    }
}


float rm_angle_deg;
float v_aim;

void controlTask(void *argument) {
    TickType_t currentTime = xTaskGetTickCount();
    TickType_t rcUpdateTime = currentTime;
    constexpr TickType_t kRemoteInputTimeout = pdMS_TO_TICKS(500);
    bool remote_received = false;
    bool sc_initialized = false;
    bool sc_was_middle = false;


    controlInit();
    // uint32_t last_time = HAL_GetTick();
    control_rm_cmd.joyLHori = kJoyCenter;
    control_rm_cmd.joyLVert = kJoyCenter;
    control_rm_cmd.joyRHori = kJoyCenter;
    control_rm_cmd.joyRVert = kJoyCenter;
    for (;;) {
        // static float controlTask_dt = 0.0f;
        // static uint32_t controlTask_DWT_CNT = 0;
        // controlTask_dt = DWT_GetDeltaT(&controlTask_DWT_CNT);

        //test begin
        const TickType_t now = xTaskGetTickCount();
        while (control_rm_sub.TryGet(&control_rm_cmd)) {
            rcUpdateTime = now;
            remote_received = true;
        }
        if (remote_received && now - rcUpdateTime <= kRemoteInputTimeout) {
            const bool sc_middle =
                control_rm_cmd.swC == RM_3_POS_SW_State_t::MIDDLE;
            if (sc_initialized && sc_middle && !sc_was_middle) {
                requestArmReleaseKFS();
            }
            sc_was_middle = sc_middle;
            sc_initialized = true;
        } else {
            sc_initialized = false;
        }
        const bool auto_mode = control_rm_cmd.swD == RM_2_POS_SW_State_t::DOWN;
        uint8_t test_sequence_id = 255U;
        if (auto_mode && remote_received &&
            now - rcUpdateTime <= kRemoteInputTimeout) {
            switch (control_rm_cmd.swB) {
                case RM_3_POS_SW_State_t::UP:     test_sequence_id = 0U; break;
                case RM_3_POS_SW_State_t::MIDDLE: test_sequence_id = 1U; break;
                case RM_3_POS_SW_State_t::DOWN:   test_sequence_id = 2U; break;
                default: break;
            }
        }
        const bool remote_fresh = remote_received &&
            now - rcUpdateTime <= kRemoteInputTimeout;
        if (!remote_received || now - rcUpdateTime > kRemoteInputTimeout) {
            control_rm_cmd.joyLHori = kJoyCenter;
            control_rm_cmd.joyLVert = kJoyCenter;
            control_rm_cmd.joyRHori = kJoyCenter;
            control_rm_cmd.joyRVert = kJoyCenter;

        }
        /*
        {
            rm_cmd.linear_x_ = JoyToVelocity(control_rm_cmd.joyLHori, kJoyDeadZoneLeft, MAX_VELOCITY_LINEAR);
            rm_cmd.linear_y_ = JoyToVelocity(control_rm_cmd.joyLVert, kJoyDeadZoneLeft, MAX_VELOCITY_LINEAR);
            rm_cmd.omega_    = -JoyToVelocity(control_rm_cmd.joyRHori, kJoyDeadZoneRight, MAX_VELOCITY_ANGULAR);

            rm_angle_deg = atan2(rm_cmd.linear_y_, rm_cmd.linear_x_) / kDegToRad;
            v_aim = sqrt(rm_cmd.linear_x_ * rm_cmd.linear_x_ + rm_cmd.linear_y_ * rm_cmd.linear_y_);
            robot_v_aim_cmd.linear_x_ = v_aim * cos((rm_angle_deg - state_now_cmd.omega_) * kDegToRad);
            robot_v_aim_cmd.linear_y_ = v_aim * sin((rm_angle_deg - state_now_cmd.omega_) * kDegToRad);
            robot_v_aim_cmd.omega_ = rm_cmd.omega_;    
        }

        U8_3.setMotorCtrl(U8_3_erpm, VESCMotor::VESC_MODE::SET_ERPM);

        chassis_data_pub.Publish(robot_v_aim_cmd);
        */

        const arm_sequence_cmd arm_request{
            test_sequence_id,
            remote_fresh && control_rm_cmd.swA == RM_2_POS_SW_State_t::DOWN,
            arm_release_count.load(std::memory_order_relaxed)
        };
        arm_sequence_cmd_pub.Publish(arm_request);

        vTaskDelayUntil(&currentTime, pdMS_TO_TICKS(5));
    }
}
