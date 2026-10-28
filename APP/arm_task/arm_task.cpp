#include "arm_task.hpp"
#include "Motor.hpp"
#include "arm_solution.hpp"
#include "cmsis_os2.h"
#include "com_config.h"
#include "pid_controller.h"

#include "portmacro.h"
#include "topic_pool.h"
#include "topics.hpp"
#include <array>
#include <atomic>

osThreadId_t armTaskHandle;

extern DM4310Motor arm4310_motor;
extern DM4340Motor arm4340_motor;
extern Arm_DJMotor arm_rotate_motor;
extern Arm_DJMotor arm_extend_motor;
extern C610Motor arm2006_motor;
extern C620Motor arm3508_motor;

static TypedTopicSubscriber<arm_cmd>arm_cmd_sub("arm_cmd",8);
static arm_cmd arm_control_cmd{};
static arm_cmd arm_control_cmd_last{};

Arm arm(arm4340_motor, arm_rotate_motor, arm4310_motor, arm_extend_motor);

static std::atomic<bool> arm_startup_position_ready{false};

namespace {
constexpr float kStartupLiftTargetRad = kArmLiftRawZeroDeg * kArmDegreeToRad;
constexpr float kStartupFlipTargetRad = -90.0f * kArmDegreeToRad;
constexpr float kStartupPositionSpeedRadPerSec = 20.0f * kArmDegreeToRad;
constexpr float kStartupPositionToleranceRad = 0.5f * kArmDegreeToRad;
constexpr float kStartupSpeedToleranceRadPerSec = 0.05f;
}

bool armFeedbackReady()
{
    return !arm4340_motor.isOffline() && !arm4310_motor.isOffline() &&
           !arm3508_motor.isOffline() && !arm2006_motor.isOffline();
}

bool armFeedbackSampled()
{
    return arm4340_motor.getRxTimestamp() != 0U &&
           arm4310_motor.getRxTimestamp() != 0U &&
           arm3508_motor.getRxTimestamp() != 0U &&
           arm2006_motor.getRxTimestamp() != 0U;
}

bool armStartupPositionReady()
{
    return arm_startup_position_ready.load(std::memory_order_acquire);
}

Arm_Motors_AngleClassis Arm_Solution::getangles(void)
{
    // CAN interrupts update feedback; keep this four-joint snapshot consistent.
    taskENTER_CRITICAL();
    const float lift_raw_deg = arm4340_motor.getRawCurrentSumPos() * kArmRadToDegree;
    current_arm_motors_angle_.lift_pos_deg_ =
        lift_raw_deg - kArmLiftRawZeroDeg;
    current_arm_motors_angle_.rotate_pos_deg_ = arm3508_motor.getCurrentSumPos();
    current_arm_motors_angle_.flip_pos_deg_ =
        arm4310_motor.getRawCurrentSumPos() * kArmRadToDegree;
    current_arm_motors_angle_.extend_pos_deg_ = arm2006_motor.getCurrentSumPos();
    taskEXIT_CRITICAL();
    return current_arm_motors_angle_;
}

void Arm::Translator(uint8_t sequence_id, PATH& path)
{
    switch (sequence_id) {
        case 0U: path = START; break;
        case 1U: path = FETCH; break;
        case 2U: path = HOLD; break;
        default: path = INVALID_PATH; break;
    }
}

void armTask(void* argument)
{   
    TickType_t current_time = xTaskGetTickCount();
    TickType_t last_command_time = current_time;
    bool have_command = false;
    Arm_Solution feedback;
    bool startup_position_done = false;
    // Hold pose latched on entry; re-measuring it every cycle would let the
    // DM4310's gravity steady-state error ratchet the flip joint downward.
    Arm_Motors_AngleClassis hold_pose{};
    bool hold_latched = false;
    arm.releaseKFS();

    for(;;)
    {
        while(arm_cmd_sub.TryGet(&arm_control_cmd))
        {
            arm_control_cmd_last = arm_control_cmd;
            last_command_time = xTaskGetTickCount();
            have_command = true;
        }

        if (have_command &&
            xTaskGetTickCount() - last_command_time <= pdMS_TO_TICKS(100)) {
            if (arm_control_cmd_last.enable && arm_control_cmd_last.suction_on) {
                arm.fetchKFS();
            } else {
                arm.releaseKFS();
            }
        }

        // On power-up, move only the two DM joints to their requested angles.
        // Wait for both first feedback frames before sending position commands.
        if (!startup_position_done) {
            const bool lift_sampled = arm4340_motor.getRxTimestamp() != 0U;
            const bool flip_sampled = arm4310_motor.getRxTimestamp() != 0U;
            if (lift_sampled && flip_sampled) {
                arm4340_motor.posWithSpeedControl(
                    kStartupLiftTargetRad, kStartupPositionSpeedRadPerSec);
                arm4310_motor.posWithSpeedControl(
                    kStartupFlipTargetRad, kStartupPositionSpeedRadPerSec);

                const bool lift_in_position =
                    std::fabs(arm4340_motor.getRawCurrentSumPos() -
                              kStartupLiftTargetRad) <= kStartupPositionToleranceRad &&
                    std::fabs(arm4340_motor.getRawCurrentSpeed()) <=
                              kStartupSpeedToleranceRadPerSec;
                const bool flip_in_position =
                    std::fabs(arm4310_motor.getRawCurrentSumPos() -
                              kStartupFlipTargetRad) <= kStartupPositionToleranceRad &&
                    std::fabs(arm4310_motor.getRawCurrentSpeed()) <=
                              kStartupSpeedToleranceRadPerSec;
                if (lift_in_position && flip_in_position) {
                    startup_position_done = true;
                    // Discard commands computed before startup positioning finished.
                    have_command = false;
                    arm_startup_position_ready.store(true, std::memory_order_release);
                }
            }
        }

        if (startup_position_done && have_command && arm_control_cmd_last.enable &&
           xTaskGetTickCount() - last_command_time <= pdMS_TO_TICKS(100))
        {
            hold_latched = false;
            arm.setLift(arm_control_cmd_last.lift_pos, arm_control_cmd_last.lift_speed);
            arm.setRotate(arm_control_cmd_last.rotate_pos, arm_control_cmd_last.rotate_speed);
            arm.setFlip(arm_control_cmd_last.flip_pos, arm_control_cmd_last.flip_speed);
            arm.setExtend(arm_control_cmd_last.extend_pos, arm_control_cmd_last.extend_speed);
        }

        else if (startup_position_done && armFeedbackReady()) {
            if (!hold_latched) {
                hold_pose = feedback.getangles();
                hold_latched = true;
            }
            arm.setLift(hold_pose.lift_pos_deg_, 5);
            arm.setRotate(hold_pose.rotate_pos_deg_, 5);
            arm.setFlip(hold_pose.flip_pos_deg_, 5);
            arm.setExtend(hold_pose.extend_pos_deg_, 5);

        }

        vTaskDelayUntil(&current_time,pdMS_TO_TICKS(5));
    }
    
}
