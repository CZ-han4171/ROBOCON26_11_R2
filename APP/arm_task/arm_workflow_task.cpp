#include "arm_workflow_task.hpp"

#include "arm_task.hpp"
#include "arm_workflow.hpp"
#include "topic_pool.h"
#include "topics.hpp"

#include <cmath>

namespace {
constexpr TickType_t kPeriod = pdMS_TO_TICKS(5);
constexpr TickType_t kRequestTimeout = pdMS_TO_TICKS(100);
constexpr float kPeriodSeconds = 0.005f;
constexpr float kMotorSpeedDegPerSec = 1500.0f;

TypedTopicSubscriber<arm_sequence_cmd> request_sub("arm_sequence_cmd", 8);
TypedTopicPublisher<arm_cmd> motor_cmd_pub("arm_cmd");
Arm_Solution solver;
ArmWorkflow workflow;
}  // namespace

void armWorkflowTask(void* argument)
{
    (void)argument;
    TickType_t last_wake = xTaskGetTickCount();
    TickType_t last_request_time = last_wake;
    arm_sequence_cmd request{255U, false, 0U};
    arm_cmd motor_cmd{};
    bool have_request = false;
    bool control_was_enabled = false;
    uint32_t last_release_count = 0U;

    for (;;) {
        arm_sequence_cmd incoming{};
        while (request_sub.TryGet(&incoming)) {
            request = incoming;
            last_request_time = xTaskGetTickCount();
            have_request = true;
        }

        const bool request_fresh = have_request &&
            xTaskGetTickCount() - last_request_time <= kRequestTimeout;
        const bool feedback_sampled = armFeedbackSampled();
        const bool startup_ready = armStartupPositionReady();
        const auto measured = solver.getangles();
        const bool measured_valid =
            std::isfinite(measured.lift_pos_deg_) &&
            std::isfinite(measured.rotate_pos_deg_) &&
            std::isfinite(measured.flip_pos_deg_) &&
            std::isfinite(measured.extend_pos_deg_);

        motor_cmd.enable = request_fresh && request.enable &&
            startup_ready && feedback_sampled && measured_valid;
        PATH requested_path = INVALID_PATH;
        if (motor_cmd.enable)
            Arm::Translator(request.sequence_id, requested_path);

        // Do not capture the pre-startup FLIP angle as the first hold target.
        if (measured_valid && feedback_sampled && startup_ready)
            workflow.update(solver, measured, requested_path, kPeriodSeconds);
        else if (control_was_enabled)
            workflow.update(solver, workflow.target(), INVALID_PATH, kPeriodSeconds);

        if (have_request && request.release_count != last_release_count) {
            last_release_count = request.release_count;
            workflow.releaseSuction();
        }
        control_was_enabled = motor_cmd.enable;

        const auto& target = workflow.target();
        motor_cmd.lift_pos = target.lift_pos_deg_;
        motor_cmd.rotate_pos = target.rotate_pos_deg_;
        motor_cmd.flip_pos = target.flip_pos_deg_;
        motor_cmd.extend_pos = target.extend_pos_deg_;
        motor_cmd.lift_speed = kMotorSpeedDegPerSec;
        motor_cmd.rotate_speed = kMotorSpeedDegPerSec;
        motor_cmd.flip_speed = kMotorSpeedDegPerSec;
        motor_cmd.extend_speed = kMotorSpeedDegPerSec;
        motor_cmd.suction_on = workflow.suctionOn();
        motor_cmd_pub.Publish(motor_cmd);

        vTaskDelayUntil(&last_wake, kPeriod);
    }
}
