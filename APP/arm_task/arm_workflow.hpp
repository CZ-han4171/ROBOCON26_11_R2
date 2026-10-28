#pragma once

#include "arm_solution.hpp"
#include <algorithm>
#include <cmath>

// Joint positions and velocities here are in degrees and degrees/second.
class ArmWorkflow {
public:
    // Keep the existing telemetry values: 1 was the removed manual state.
    enum class State : uint8_t { Hold = 0, Running = 2, Complete = 3, Fault = 4 };
    enum class Fault : uint8_t { None, InvalidPath, NoIK, TrackingTimeout };
    enum class FailureReason : uint8_t {
        None, InvalidPath, IKInvalid, IKNonFinite, JointStepExceeded, TrackingTimeout
    };

    void update(Arm_Solution& solver, const Arm_Motors_AngleClassis& measured,
                PATH requested_path, float dt)
    {
        if (!target_initialized_) {
            target_ = measured;
            target_initialized_ = true;
        }
        const bool valid_path = requested_path == START || requested_path == FETCH ||
                                requested_path == HOLD;
        const bool start_requested = valid_path;
        const bool start_edge = start_requested && !switch_was_down_;
        switch_was_down_ = start_requested;

        // No requested path stops progress and holds the last commanded pose.
        if (!start_requested) {
            if (state_ == State::Running) state_ = State::Hold;
            return;
        }
        if (start_edge) {
            target_ = measured;
            path_start_ = solver.calculateAnglestoVP(measured);
            active_path_ = requested_path;
            segment_ = 0;
            progress_ = 0.0f;
            wait_time_ = 0.0f;
            fetch_orienting_ = requested_path == FETCH;
            if (requested_path == START) suction_on_ = false;
            gripping_ = false;
            fetch_start_lift_deg_ = measured.lift_pos_deg_;
            fetch_initial_direction_deg_ = path_start_.VP_direction_deg;
            state_ = State::Running;
            fault_ = Fault::None;
            failure_reason_ = FailureReason::None;
            last_ik_step_error_ = 0.0f;
        }

        if (state_ != State::Running) return;

        // Pace nominal segment timing to the real motor response.
        const float tolerance = 80.0f;
        if (maxError(measured, target_) > tolerance) {
            wait_time_ += dt;
            if (wait_time_ >= 3.0f)
                fail(Fault::TrackingTimeout, FailureReason::TrackingTimeout, measured);
            return;
        }
        wait_time_ = 0.0f;
        if (gripping_) {
            grip_time_ += dt;
            if (grip_time_ < 0.5f) return;
            gripping_ = false;
            state_ = State::Complete;
            return;
        }
        if (active_path_ == HOLD) {
            if (progress_ >= 1.0f) {
                if (maxError(measured, target_) <= 5.0f) {
                    state_ = State::Complete;
                } else {
                    settle_time_ += dt;
                    if (settle_time_ >= 3.0f)
                        fail(Fault::TrackingTimeout, FailureReason::TrackingTimeout, measured);
                }
                return;
            }
            
            const float next_progress = std::min(1.0f, progress_ + dt / 6.0f);
            const float t = next_progress * next_progress * (3.0f - 2.0f * next_progress);
            VPposition arc_target{};
            if (!solver.PathOrganiser(HOLD, path_start_, 0, t, arc_target)) {
                fail(Fault::InvalidPath, FailureReason::InvalidPath, measured);
                return;
            }
            const auto result = solver.calculateVPtoAngles(arc_target, target_, 10.0f, -1.0f);
            if (!result.isValid) {
                fail(Fault::NoIK, FailureReason::IKInvalid, measured);
                return;
            }
            const auto& next = result.AnglesClassis;
            last_ik_step_error_ = maxError(next, target_);
            if (last_ik_step_error_ > 10.0f) {
                fail(Fault::NoIK, FailureReason::JointStepExceeded, measured);
                return;
            }
            target_ = next;
            progress_ = next_progress;
            return;
        }
        if (progress_ >= 1.0f && !fetch_orienting_) {
            const uint32_t segment_count = active_path_ == START
                ? static_cast<uint32_t>(sizeof(start_path) / sizeof(start_path[0]))
                : static_cast<uint32_t>(sizeof(fetch_path) / sizeof(fetch_path[0]));
            
            if (active_path_ == START && segment_ + 1 >= segment_count &&
                (fabsf(measured.rotate_pos_deg_ - target_.rotate_pos_deg_) > 3.0f ||
                 fabsf(measured.flip_pos_deg_ - target_.flip_pos_deg_) > 3.0f)) {
                settle_time_ += dt;
                if (settle_time_ >= 3.0f)
                    fail(Fault::TrackingTimeout, FailureReason::TrackingTimeout, measured);
                return;
            }
            if (segment_ + 1 >= segment_count) {
                const float terminal_error = active_path_ == START ? 5.0f : 8.0f;
                if (maxError(measured, target_) > terminal_error ||
                    fabsf(measured.rotate_pos_deg_ - target_.rotate_pos_deg_) > 3.0f ||
                    fabsf(measured.flip_pos_deg_ - target_.flip_pos_deg_) > 3.0f) {
                    settle_time_ += dt;
                    if (settle_time_ >= 5.0f)
                        fail(Fault::TrackingTimeout, FailureReason::TrackingTimeout, measured);
                    return;
                }
                settle_time_ = 0.0f;
                if (active_path_ == FETCH) {
                    suction_on_ = true;
                    gripping_ = true;
                    grip_time_ = 0.0f;
                } else {
                    state_ = State::Complete;
                }
                return;
            }
            ++segment_;
            progress_ = 0.0f;
        }

        if (active_path_ == FETCH && fetch_orienting_) {
           
            if (progress_ >= 1.0f) {
                if (fabsf(measured.rotate_pos_deg_ - target_.rotate_pos_deg_) > 3.0f ||
                    fabsf(measured.flip_pos_deg_ - target_.flip_pos_deg_) > 3.0f) {
                    settle_time_ += dt;
                    if (settle_time_ >= 3.0f)
                        fail(Fault::TrackingTimeout, FailureReason::TrackingTimeout, measured);
                    return;
                }
                path_start_ = solver.calculateAnglestoVP(target_);
                fetch_start_lift_deg_ = target_.lift_pos_deg_;
                fetch_orienting_ = false;
                progress_ = 0.0f;
                settle_time_ = 0.0f;
                return;
            }
            const float next_progress = std::min(1.0f, progress_ + dt / 2.0f);
            const float t = next_progress * next_progress * (3.0f - 2.0f * next_progress);
            
            VPposition next = path_start_;

            next.VP_direction_deg = fetch_initial_direction_deg_ +
                (kFetchEndDirectionDeg - fetch_initial_direction_deg_) * t;
            const auto result = solver.calculateVPtoAnglesAtLift(next, fetch_start_lift_deg_);
            if (!result.isValid) {
                fail(Fault::NoIK, FailureReason::IKInvalid, measured);
                return;
            }
            last_ik_step_error_ = maxError(result.AnglesClassis, target_);
            if (last_ik_step_error_ > 10.0f) {
                fail(Fault::NoIK, FailureReason::JointStepExceeded, measured);
                return;
            }
            target_ = result.AnglesClassis;
            progress_ = next_progress;
            return;
        }

        const float duration = active_path_ == FETCH ? 5.0f
            : (segment_ == 0 ? 1.0f : 2.5f);
        const float next_progress = std::min(1.0f, progress_ + dt / duration);
        const float t = next_progress * next_progress * (3.0f - 2.0f * next_progress);
        VPposition next{};
        if (!solver.PathOrganiser(active_path_, path_start_, segment_, t, next)) {
            fail(Fault::InvalidPath, FailureReason::InvalidPath, measured);
            return;
        }

        const float min_lift_deg = -1.0f;
        IKResult result{};
        if (active_path_ == FETCH) {
            result = solver.calculateVPtoAnglesAtLift(
                next, fetch_start_lift_deg_ +
                    (kFetchEndLiftDeg - fetch_start_lift_deg_) * t);
        } 
        else {
            // Anchor IK to the last command so lagging feedback cannot switch
            // branches while the Cartesian target advances smoothly.
            result = solver.calculateVPtoAngles(next, target_, 80.0f, min_lift_deg);
        }
        
        if (!result.isValid) {
            fail(Fault::NoIK, FailureReason::IKInvalid, measured);
            return;
        }
        last_ik_step_error_ = maxError(result.AnglesClassis, target_);
        if (last_ik_step_error_ > 10.0f) {
            fail(Fault::NoIK, FailureReason::JointStepExceeded, measured);
            return;
        }
        target_ = result.AnglesClassis;
        progress_ = next_progress;
    }

    const Arm_Motors_AngleClassis& target() const { return target_; }
    State state() const { return state_; }
    Fault fault() const { return fault_; }
    FailureReason failureReason() const { return failure_reason_; }
    uint32_t segment() const { return segment_; }
    PATH path() const { return active_path_; }
    float progress() const { return progress_; }
    bool suctionOn() const { return suction_on_; }
    void releaseSuction() {
        suction_on_ = false;
        gripping_ = false;
        if (state_ == State::Running) state_ = State::Hold;
    }

private:
    static float maxError(const Arm_Motors_AngleClassis& a, const Arm_Motors_AngleClassis& b) {
        return std::max({fabsf(a.lift_pos_deg_ - b.lift_pos_deg_),
            fabsf(a.rotate_pos_deg_ - b.rotate_pos_deg_), fabsf(a.flip_pos_deg_ - b.flip_pos_deg_),
            fabsf(a.extend_pos_deg_ - b.extend_pos_deg_)});
    }
    void fail(Fault fault, FailureReason reason, const Arm_Motors_AngleClassis& measured) {
        target_ = measured;
        fault_ = fault;
        failure_reason_ = reason;
        state_ = State::Fault;
    }
    Arm_Motors_AngleClassis target_{};
    float fetch_start_lift_deg_{0.0f};
    float fetch_initial_direction_deg_{0.0f};
    bool fetch_orienting_{false};
    bool suction_on_{false};
    bool gripping_{false};
    float grip_time_{0.0f};
    VPposition path_start_{};
    PATH active_path_{START};
    State state_{State::Hold};
    Fault fault_{Fault::None};
    FailureReason failure_reason_{FailureReason::None};
    bool target_initialized_{false};
    bool switch_was_down_{false};
    uint32_t segment_{0};
    float progress_{0.0f};
    float wait_time_{0.0f};
    float settle_time_{0.0f};
    float last_ik_step_error_{0.0f};

};
