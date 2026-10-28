#include "arm_solution.hpp"

namespace {
constexpr float kLiftMinDeg = -1.0f;
constexpr float kLiftMaxDeg = 1200.0f;
constexpr float kRotateMinDeg = -180.0f;
constexpr float kRotateMaxDeg = 180.0f;
constexpr float kFlipMinDeg = -120.0f;
constexpr float kFlipMaxDeg = 125.0f;
constexpr float kExtendMinDeg = 0.0f;
constexpr float kExtendMaxDeg = 1200.0f;
// The former tail solution lay directly on the forbidden X boundary.
// Leave room for geometry and tracking error; this margin applies to the tail only.
constexpr float kTailForbiddenClearanceMm = 120.0f;
} 

VPposition Arm_Solution::calculateAnglestoVP(const Arm_Motors_AngleClassis &arm_motors_angle)
{
    VPposition result{};

    const float lift_distance = arm_motors_angle.lift_pos_deg_ * kDISTANCE_2_DEG_withoutD * lift_D;   //mm
    const float extend_distance = arm_motors_angle.extend_pos_deg_ * kDISTANCE_2_DEG_withoutD * EXTEND_D; //mm 
    const float arm_direction = arm_motors_angle.rotate_pos_deg_ * kArmRotateGeometryRatio;
    const float vp_direction = arm_motors_angle.rotate_pos_deg_ * kArmRotateGeometryRatio +
        arm_motors_angle.flip_pos_deg_ * kArmFlipGeometryRatio;

    result.y = lift_distance + cosf(arm_direction * kArmDegreeToRad) * (extend_distance + DJ3508toDM4310_distance_y) + cosf(vp_direction * kArmDegreeToRad) * VPtoDM4310_distance + DM4310toDJ3508_distance_y + kArmYOriginOffsetMm;
    result.x = -sinf(arm_direction * kArmDegreeToRad) * (extend_distance + DJ3508toDM4310_distance_y) - sinf(vp_direction * kArmDegreeToRad) * VPtoDM4310_distance + DM4310toDJ3508_distance_x;
    result.VP_direction_deg = vp_direction;

    return result;

}

float Arm_Solution::valueVP(const Arm_Motors_AngleClassis& current, const Arm_Motors_AngleClassis& target)
{
    float l_cost = fabs(current.lift_pos_deg_   - target.lift_pos_deg_);
    float r_cost = fabs(current.rotate_pos_deg_ - target.rotate_pos_deg_);
    float e_cost = fabs(current.extend_pos_deg_ - target.extend_pos_deg_);
    float f_cost = fabs(current.flip_pos_deg_   - target.flip_pos_deg_);
    

    float cost = l_cost + 2 * (r_cost + e_cost + f_cost);

    return cost;
}

IKResult Arm_Solution::calculateVPtoAngles(VPposition VP, float max_step_deg)
{
    return calculateVPtoAngles(VP, getangles(), max_step_deg);
}

IKResult Arm_Solution::calculateVPtoAngles(VPposition VP,
    const Arm_Motors_AngleClassis& measured, float max_step_deg,
    float min_lift_deg)
{
    VP_ = VP;
    const float theta = VP_.VP_direction_deg * kArmDegreeToRad;
    
    const auto current = measured;
    IKResult output{current, 0.0f, false};
    if (!std::isfinite(max_step_deg) || max_step_deg < 0.0f ||
        !std::isfinite(min_lift_deg) ||
        !std::isfinite(VP.x) || !std::isfinite(VP.y) ||
        !std::isfinite(VP.VP_direction_deg) ||
        !std::isfinite(current.lift_pos_deg_) || !std::isfinite(current.rotate_pos_deg_) ||
        !std::isfinite(current.flip_pos_deg_) || !std::isfinite(current.extend_pos_deg_)) return output;
    const auto previousVP = calculateAnglestoVP(current);
    const float previousLift =
        current.lift_pos_deg_ * kDISTANCE_2_DEG_withoutD * lift_D;

    // All lift candidates use the same inverse, validation and cost checks.
    const auto tryLift = [&](float lift_distance) {
        const float arm_x = VP_.x - DM4310toDJ3508_distance_x + VPtoDM4310_distance * sinf(theta);
        const float arm_y = VP_.y - DM4310toDJ3508_distance_y - kArmYOriginOffsetMm - lift_distance - VPtoDM4310_distance * cosf(theta);

        const float arm_distance = hypotf(arm_x, arm_y);
        // The forward model includes the fixed pivot-to-flip offset.
        // Positions inside that radius would require a negative extension.
        if (arm_distance < DJ3508toDM4310_distance_y - 0.001f) return;
        const float extend_distance = fmaxf(0.0f,
            arm_distance - DJ3508toDM4310_distance_y);
        arm_motors_angle_.rotate_pos_deg_ =
            atan2f(-arm_x, arm_y) * kArmRadToDegree / kArmRotateGeometryRatio;

        if(arm_motors_angle_.rotate_pos_deg_ == 360 || arm_motors_angle_.rotate_pos_deg_ == -360)
        {
            arm_motors_angle_.rotate_pos_deg_ = 0;
        }

        if(arm_motors_angle_.rotate_pos_deg_ == 180)
        {
            arm_motors_angle_.rotate_pos_deg_ = -180;
        }
        

        arm_motors_angle_.flip_pos_deg_ =
            (VP_.VP_direction_deg -
             arm_motors_angle_.rotate_pos_deg_ * kArmRotateGeometryRatio) /
            kArmFlipGeometryRatio;

        arm_motors_angle_.lift_pos_deg_ = lift_distance / (kDISTANCE_2_DEG_withoutD * lift_D);
        arm_motors_angle_.extend_pos_deg_ = extend_distance / (kDISTANCE_2_DEG_withoutD * EXTEND_D);
        
        if (arm_motors_angle_.lift_pos_deg_ < min_lift_deg ||
            !isValid(arm_motors_angle_, VP) ||
            fabsf(arm_motors_angle_.lift_pos_deg_ - current.lift_pos_deg_) > max_step_deg ||
            fabsf(arm_motors_angle_.rotate_pos_deg_ - current.rotate_pos_deg_) > max_step_deg ||
            fabsf(arm_motors_angle_.flip_pos_deg_ - current.flip_pos_deg_) > max_step_deg ||
            fabsf(arm_motors_angle_.extend_pos_deg_ - current.extend_pos_deg_) > max_step_deg) {
            return;
        }
        const float cost = Arm_Solution::valueVP(current,arm_motors_angle_);
        
        if(!output.isValid || output.cost > cost)
        {
            output.AnglesClassis = arm_motors_angle_;
            output.cost = cost;
            output.isValid = true;
        }
    };

    tryLift(previousLift);
    tryLift(previousLift + (VP_.y - previousVP.y));

    const float lift_min = fmaxf(fmaxf(kLiftMinDeg, min_lift_deg),
                                 current.lift_pos_deg_ - max_step_deg);
    const float lift_max = fminf(kLiftMaxDeg, current.lift_pos_deg_ + max_step_deg);
    if (lift_min <= lift_max) {
        for (uint32_t i = 0; i <= 64; ++i) {
            const float lift_deg = lift_min + (lift_max - lift_min) *
                (static_cast<float>(i) / 64.0f);
            tryLift(lift_deg * kDISTANCE_2_DEG_withoutD * lift_D);
        }
    }

    const uint32_t samples = 1500;
    for (uint32_t i = 0; i <= samples; i++) {
        const float kLiftaccount =
            1.5f * static_cast<float>(samples - i) / static_cast<float>(samples);
        tryLift(kLiftaccount * VP_.y);
    }
    return output;
}

IKResult Arm_Solution::calculateVPtoAnglesAtLift(VPposition VP, float lift_deg)
{
    IKResult result{{}, 0.0f, false};
    if (!std::isfinite(VP.x) || !std::isfinite(VP.y) ||
        !std::isfinite(VP.VP_direction_deg) || !std::isfinite(lift_deg)) return result;

    const float theta = VP.VP_direction_deg * kArmDegreeToRad;
    const float lift_distance = lift_deg * kDISTANCE_2_DEG_withoutD * lift_D;
    const float arm_x = VP.x - DM4310toDJ3508_distance_x +
        VPtoDM4310_distance * sinf(theta);
    const float arm_y = VP.y - DM4310toDJ3508_distance_y - kArmYOriginOffsetMm - lift_distance -
        VPtoDM4310_distance * cosf(theta);
    const float arm_distance = hypotf(arm_x, arm_y);
    if (arm_distance < DJ3508toDM4310_distance_y - 0.001f) return result;

    auto& q = result.AnglesClassis;
    q.lift_pos_deg_ = lift_deg;
    q.rotate_pos_deg_ = atan2f(-arm_x, arm_y) * kArmRadToDegree /
        kArmRotateGeometryRatio;
    q.flip_pos_deg_ = (VP.VP_direction_deg -
        q.rotate_pos_deg_ * kArmRotateGeometryRatio) / kArmFlipGeometryRatio;
    q.extend_pos_deg_ = fmaxf(0.0f, arm_distance - DJ3508toDM4310_distance_y) /
        (kDISTANCE_2_DEG_withoutD * EXTEND_D);
    result.isValid = isValid(q, VP);
    return result;
}

bool Arm_Solution::circleFromThreePoints(const position& from, const position& via,
                                         const position& end, position& center)
{
    const float det = 2.0f * (from.x * (via.y - end.y) +
        via.x * (end.y - from.y) +
        end.x * (from.y - via.y));
    if (fabsf(det) < 1.0f) return false;
    const float a2 = from.x * from.x + from.y * from.y;
    const float b2 = via.x * via.x + via.y * via.y;
    const float c2 = end.x * end.x + end.y * end.y;
    center.x = (a2 * (via.y - end.y) +
        b2 * (end.y - from.y) +
        c2 * (from.y - via.y)) / det;
    center.y = (a2 * (end.x - via.x) +
        b2 * (from.x - end.x) +
        c2 * (via.x - from.x)) / det;
    return true;
}

// 420 80   //-50 100  -120 620
bool Arm_Solution::isValid(const Arm_Motors_AngleClassis& angles, const VPposition& VP)
{
    if (!std::isfinite(angles.lift_pos_deg_) || !std::isfinite(angles.rotate_pos_deg_) ||
        !std::isfinite(angles.flip_pos_deg_) || !std::isfinite(angles.extend_pos_deg_)) return false;
    const float arm_direction = angles.rotate_pos_deg_ * kArmRotateGeometryRatio * kArmDegreeToRad;
    const float tail_distance = totalDistance -
        (DJ3508toDM4310_distance_y +
         angles.extend_pos_deg_ * kDISTANCE_2_DEG_withoutD * EXTEND_D);
    Arm_Solution::bottom.y = angles.lift_pos_deg_ * kDISTANCE_2_DEG_withoutD * lift_D -
        cosf(arm_direction) * tail_distance + DM4310toDJ3508_distance_y +
        kArmYOriginOffsetMm;
    Arm_Solution::bottom.x = sinf(arm_direction) * tail_distance + DM4310toDJ3508_distance_x;

    // The rear shaft can enter the forbidden X side even above the old
    // Y=620 cut-off while ROTATE swings. Keep the entire tail point clear.
    if (Arm_Solution::bottom.x <= -120.0f + kArmXCalibrationOffsetMm +
            kTailForbiddenClearanceMm) {
        return false;
    }

    if((Arm_Solution::bottom.y <= 100 && Arm_Solution::bottom.x <= -50 + kArmXCalibrationOffsetMm) ||
       (VP.y <= 100 && VP.x <= -50 + kArmXCalibrationOffsetMm))
    {
        return false;
    }

    else if((Arm_Solution::bottom.y >= 100 &&Arm_Solution::bottom.y <= 620 &&
             Arm_Solution::bottom.x <= -120 + kArmXCalibrationOffsetMm + kTailForbiddenClearanceMm) ||
            (VP.y >= 100 && VP.y <= 620 && VP.x <= -120 + kArmXCalibrationOffsetMm))
    {
        return false;
    }
    else if(angles.lift_pos_deg_ < kLiftMinDeg ||
            angles.lift_pos_deg_ > kLiftMaxDeg ||
            angles.rotate_pos_deg_ < kRotateMinDeg ||
            angles.rotate_pos_deg_ > kRotateMaxDeg ||
            angles.flip_pos_deg_ < kFlipMinDeg ||
            angles.flip_pos_deg_ > kFlipMaxDeg ||
            angles.extend_pos_deg_ < kExtendMinDeg ||
            angles.extend_pos_deg_ > kExtendMaxDeg)
    {
        return false;
    }
    else
    {
        return true;
    }
}

bool Arm_Solution::PathOrganiser(PATH pathsign, const VPposition& path_start,
                                 uint32_t segment, float progress, VPposition& output)
{
    const PathSegment* path = nullptr;
    uint32_t count = 0;
    switch (pathsign) {
    case START:
        path = start_path;
        count = static_cast<uint32_t>(sizeof(start_path) / sizeof(start_path[0]));
        break;
    case FETCH:
        path = fetch_path;
        count = static_cast<uint32_t>(sizeof(fetch_path) / sizeof(fetch_path[0]));
        break;
    case HOLD:
        path = hold_path;
        count = static_cast<uint32_t>(sizeof(hold_path) / sizeof(hold_path[0]));
        break;
    default:
        return false;
    }
    if (segment >= count || !std::isfinite(progress)) return false;

    const PathSegment& current = path[segment];
    const position from = segment == 0
        ? position{path_start.x, path_start.y}
        : path[segment - 1].end;
    const float t = progress < 0.0f ? 0.0f : (progress > 1.0f ? 1.0f : progress);   //防止超程

    VPposition next = path_start;
    if (current.curve == PathCurve::Line)
    {
        next.x = from.x + (current.end.x - from.x) * t;
        next.y = from.y + (current.end.y - from.y) * t;
    } 
    else if(current.curve == PathCurve::Arc)
    {
        position center = current.center;
        if (pathsign == FETCH) {
            const position via{
                (from.x + current.end.x) * 0.5f + kFetchArcBulgeMm,
                (from.y + current.end.y) * 0.5f};
            if (!circleFromThreePoints(from, via, current.end, center))
                return false;
        } else if (pathsign == HOLD) {
            // Derive the circle from the actual FETCH endpoint, the via point
            // and the HOLD endpoint so small IK errors do not break the arc.
            if (!circleFromThreePoints(from, kHoldArcVia, current.end, center))
                return false;
        }
        const float radius = hypotf(from.x - center.x, from.y - center.y);
        const float end_radius = hypotf(current.end.x - center.x, current.end.y - center.y);
        if (radius <= 0.0f || fabsf(radius - end_radius) > 0.1f) return false;

        const float start_angle = atan2f(from.y - center.y, from.x - center.x);
        const float end_angle = atan2f(current.end.y - center.y,
                                       current.end.x - center.x);
        float sweep = end_angle - start_angle;
        if (current.counter_clockwise && sweep < 0.0f) sweep += 2.0f * kPI;
        if (!current.counter_clockwise && sweep > 0.0f) sweep -= 2.0f * kPI;

        next.x = center.x + radius * cosf(start_angle + sweep * t);
        next.y = center.y + radius * sinf(start_angle + sweep * t);
    }
    if (pathsign == FETCH) {
        next.VP_direction_deg = path_start.VP_direction_deg +
            (kFetchEndDirectionDeg - path_start.VP_direction_deg) * t;
    } else if (pathsign == HOLD) {
        const float end_direction = -90.0f;
        next.VP_direction_deg = path_start.VP_direction_deg +
            (end_direction - path_start.VP_direction_deg) * t;
    }
    next.process = t;
    output = next;
    return true;
}
