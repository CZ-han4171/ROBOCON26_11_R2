#pragma once

#include "FreeRTOS.h"
#include "arm_solution.hpp"
#include "cmsis_os.h"
#include "gpio.h"

#include "task.h"
#include "arm_djmotor.hpp"
#include "arm_actions_config.hpp"
#include "arm_solution.hpp"
#include "topics.hpp"
#include "topic_pool.h"

void armTask(void* argument);
bool armFeedbackReady();
bool armFeedbackSampled();
bool armStartupPositionReady();

#define VP_GPIO_Port1 GPIOG
#define VP_Pin1 GPIO_PIN_3
#define VP_GPIO_Port2 GPIOG
#define VP_Pin2 GPIO_PIN_8


enum class LOAD_TYPE : int8_t {
    LOAD_LOW    = -1,
    LOAD_PLAIN  = 0,
    LOAD_MEDIUM = 1,
    LOAD_HIGH   = 2,

};

enum class UNLOAD_TYPE : uint8_t {
    UNLOAD_ING = 0,
    UNLOAD_LOW = 1,
    UNLOAD_MEDIUM = 2,
    UNLOAD_HIGH = 4,
};


class Arm{
    public:
    Arm(DM4310Motor &motor_lift, Arm_DJMotor &motor_rotate, DM4310Motor &motor_flip, Arm_DJMotor &motor_extend, uint8_t kfs_num = 0)
        : motor_lift_(motor_lift), motor_rotate_(motor_rotate), motor_flip_(motor_flip), motor_extend_(motor_extend), kfs_num_(kfs_num) {}
    ~Arm(){};

    void setLift(float pos_deg, float speed_deg_s){motor_lift_.posWithSpeedControl((pos_deg + kArmLiftRawZeroDeg) * kArmDegreeToRad, speed_deg_s * kArmDegreeToRad);};
    void setRotate(float pos, float speed){motor_rotate_.posWithSpeedControl(pos, speed * kArmDegreeToRad);}
    void setFlip(float pos_deg, float speed_deg_s){motor_flip_.posWithSpeedControl(pos_deg * kArmDegreeToRad, speed_deg_s * kArmDegreeToRad);};
    void setExtend(float pos, float speed){motor_extend_.posWithSpeedControl(pos, speed * kArmDegreeToRad);};
    
    void fetchKFS(){HAL_GPIO_WritePin(VP_GPIO_Port1, VP_Pin1, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(VP_GPIO_Port2, VP_Pin2, GPIO_PIN_SET);};
    void releaseKFS(){HAL_GPIO_WritePin(VP_GPIO_Port1, VP_Pin1, GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(VP_GPIO_Port2, VP_Pin2, GPIO_PIN_RESET);};

    void setPose(arm_pose pose)
    {
        if(!(pose.operation & VPOperation::NONE))
        {
            setLift(pose.lift.pos, pose.lift.speed);
            setRotate(pose.rotate.pos, pose.rotate.speed);
            setFlip(pose.flip.pos, pose.flip.speed);
            setExtend(pose.extend.pos, pose.extend.speed);
        }
        if(pose.operation & VPOperation::FETCH_KFS)
        {
            fetchKFS();
        }
        if(pose.operation & VPOperation::RELEASE_KFS)
        {
            releaseKFS();
        }
    }

    // Decode a sequence number received from the main controller.
    static void Translator(uint8_t sequence_id, PATH& path);
    private:
    DM4310Motor &motor_lift_;
    Arm_DJMotor &motor_rotate_;
    DM4310Motor &motor_flip_;
    Arm_DJMotor &motor_extend_;

    uint8_t kfs_num_{0};

    float now_t = 0.0f;
    float last_t = 0.0f;
    bool is_start_ = false;
    bool is_holding_ = false;
};

