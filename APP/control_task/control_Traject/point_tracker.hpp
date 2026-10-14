#pragma once

//点跟踪器
#include "control_Traject.hpp"

class PointTracker {
public:
    void SetTarget(float x_mm, float y_mm, float yaw_deg) {
        pub_chassis_cmd target{};
        target.linear_x_ = x_mm/1000.0f;//目标 x，单位：mm
        target.linear_y_ = y_mm/1000.0f;//目标 y，单位：mm
        target.omega_    = yaw_deg;   // 目标朝向，单位：度
        impl_.Set_Ref(target, speed_param_);
    }

    pub_chassis_cmd Update(float x_mm, float y_mm, float yaw_deg) {
        pub_chassis_cmd now{};
        now.linear_x_ = x_mm/1000.0f;  //当前 x，单位：mm
        now.linear_y_ = y_mm/1000.0f;  //当前 y，单位：mm
        now.omega_    = yaw_deg; //当前朝向，单位：度
        impl_.Run(now);
        return impl_.Get_output_b();
    }

    //是否已到位
    bool Arrived() const {
        return impl_.PointTrack_linear_complete_Flag &&
               impl_.PointTrack_omega_complete_Flag;
    }

private:
    TrajectChassis impl_{};  
    speed_data speed_param_{1.2f, 0.9f, 0.3f, M_PI * 0.6f};
};