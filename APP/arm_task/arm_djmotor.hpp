#pragma once

#include "Motor.hpp"
#include "pid_controller.h"
#include "cmsis_os2.h"

#include <algorithm>
#include <cmath>

extern osThreadId_t ArmDJMotorTaskHandle;
void ArmDJMotorTask(void *argument);

class Arm_DJMotor 
{
    public:
        struct PidParam {
            float kp;
            float ki;
            float kd;
            float integralLimit;
            float max_out;
            float deadband;
            uint16_t improve;

            PidParam(float kp_in = 100.0f, float ki_in = 80.0f,
                     float kd_in = 0.0f,
                     float integral_limit_in = 10000.0f,
                     float max_out_in = 20000.0f,
                     float deadband_in = 0.1f,
                     uint16_t improve_in = NONE)
                : kp(kp_in), ki(ki_in), kd(kd_in),
                  integralLimit(integral_limit_in), max_out(max_out_in),
                  deadband(deadband_in), improve(improve_in) {}
        };

        Arm_DJMotor(MotorBase &motor, const PidParam &pos_pid_param, const PidParam &speed_pid_param)
            : motor_(motor)
        {
            applyPidParam(pos_pid_,pos_pid_param);
            applyPidParam(speed_pid_,speed_pid_param);
        }

        void posWithSpeedControl(float pos_deg, float speed_rad_s)
        {
            const bool pos_changed = (std::fabs(pos_deg - target_pos_deg_)) >= 0.01f;
            const bool speed_changed = std::fabs(std::fabs(speed_rad_s) - target_speed_limit_) >= 0.01f;

            if(!pos_changed && !speed_changed){
                return;
            }

            if(pos_changed)
            {
                start_pos_deg_ = motor_.getCurrentSumPos();
                target_pos_deg_ = pos_deg;

                planned_speed_rad_s_ = 0.0f;
                pos_process_ = 0.0f;
                is_finished_ = false;
            }
            
            target_speed_limit_ = std::fabs(speed_rad_s);
        }

        void update()
        {
            current_pos_deg_ = motor_.getCurrentSumPos();

            const float current_speed_rad_s = motor_.getCurrentSpeed();

            const float pos_error_deg = target_pos_deg_ - current_pos_deg_;

            updateProcess(current_pos_deg_);

            is_finished_ =(std::fabs(pos_error_deg) <= pos_tolerance_deg_ && std::fabs(current_speed_rad_s) <= speed_tolerance_rad_s_);

            //位置环
            float pos_loop_speed = PID_Calculate(
                &pos_pid_,
                current_pos_deg_ * DEGREE_2_RAD,
                target_pos_deg_  * DEGREE_2_RAD
            );

            pos_loop_speed = std::clamp(
                pos_loop_speed,
                -target_speed_limit_,
                target_speed_limit_
            );

            //速度环
            planned_speed_rad_s_ = pos_loop_speed;

            const float current_cmd = PID_Calculate(
                &speed_pid_,
                current_speed_rad_s,
                planned_speed_rad_s_
            );

            motor_.setMotorCmd(current_cmd);
        }

        void stop() {
            target_pos_deg_ = motor_.getCurrentSumPos();
            planned_speed_rad_s_ = 0.0f;
            is_finished_ = true;

            PID_Reset(&pos_pid_);
            PID_Reset(&speed_pid_);

            motor_.setMotorCmd(0.0f);
        }

        bool isFinished() const {
            return is_finished_;
        }

    private:
        void updateProcess(float current_pos_deg) {
            const float total_distance = target_pos_deg_ - start_pos_deg_;

            if (std::fabs(total_distance) < 0.001f) {
                pos_process_ = 1.0f;
                return;
            }

            else 
            {
                pos_process_ = (current_pos_deg - start_pos_deg_) / total_distance;
            }

            pos_process_ = std::clamp(
                pos_process_,
                0.0f,
                1.0f
            );
        }

        static void applyPidParam(PID_t &pid, const PidParam &param)
        {
            pid = {};

            pid.Kp = param.kp;
            pid.Ki = param.ki;
            pid.Kd = param.kd;
            pid.IntegralLimit = param.integralLimit;
            pid.MaxOut = param.max_out;
            pid.DeadBand = param.deadband;
            pid.Improve = param.improve;

            PID_Init(&pid);
        }

        MotorBase &motor_;

        // 双环 PID
        PID_t pos_pid_{};
        PID_t speed_pid_{};

        // 位置 deg
        float start_pos_deg_{0.0f};
        float target_pos_deg_{0.0f};
        float current_pos_deg_{0.0f};

        // 速度 rad/s
        float target_speed_limit_{0.0f};
        float planned_speed_rad_s_{0.0f};

        float pos_process_{0.0f};

        const float pos_tolerance_deg_{0.2f};
        const float speed_tolerance_rad_s_{0.1f};

        bool is_finished_{true};

};
