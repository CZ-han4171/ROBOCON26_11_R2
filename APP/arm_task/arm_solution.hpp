 #pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#define VPtoDM4310_distance 11.0f //mm
#define totalDistance 650.0f
#define DM4310toDJ3508_distance_y 490.0f  // fixed pivot height at LIFT zero, mm
#define DJ3508toDM4310_distance_y 160.0f  // fixed pivot-to-FLIP length, mm
// Surveyed Y origin. This must not change either physical arm length above.
constexpr float kArmYOriginOffsetMm = 6.0f;
// Calibrate the X origin to the measured home pose without changing the arm geometry.
constexpr float kArmXCalibrationOffsetMm = 79.0f;
// Raw DM4340 angle at the mechanical LIFT zero (solver LIFT = 0 deg).
constexpr float kArmLiftRawZeroDeg = -85.0f;
#define DM4310toDJ3508_distance_x (120.0f + kArmXCalibrationOffsetMm)  //mm
// Effective LIFT travel per raw DM4340 degree, expressed as an equivalent
// diameter. Fitted from home Y~650 mm and FETCH Y~740 mm at LIFT~1100 deg.
#define lift_D 61.25f    //mm, provisional two-pose calibration
#define EXTEND_D 44.0f  //mm

#define kPI 3.1415926f
constexpr float kArmDegreeToRad = 0.01745329252f;
constexpr float kArmRadToDegree = 57.2957795f;
// Geometric angles produced by one degree of motor feedback.
constexpr float kArmRotateGeometryRatio = 1.2f;
constexpr float kArmFlipGeometryRatio = 1.0f;
constexpr float kDISTANCE_2_DEG_withoutD (kPI / 360.0f);  

 
struct Arm_Motors_AngleClassis{
    float lift_pos_deg_{0.0f};
    float rotate_pos_deg_{0.0f};
    float flip_pos_deg_{0.0f};
    float extend_pos_deg_{0.0f};
};

struct position{
    float x;
    float y;
};

struct VPposition{
    float x;
    float y;

    float VP_direction_deg;

    float process;
};

struct IKResult
{
    Arm_Motors_AngleClassis AnglesClassis;
    float cost;
    bool isValid;
};


enum PATH
{
    START = 0,
    FETCH = 1,
    HOLD = 2,
    INVALID_PATH = 255,
};

enum class VPOperation : uint8_t 
{
    NONE            =   0,
    FETCH_KFS       =   1,
    RELEASE_KFS     =   2,
};

enum class PathCurve { Line, Arc };

struct ArmPathStep
{
    PathCurve pathcurve = PathCurve::Line;
   
    float end_x_mm;

    float end_y_mm;
    float VP_Direction;

    float lift_end_deg;
 
    float arc_via_x;
    float arc_via_y;
    bool counter_clockwise;    //ture == 逆时针

    float duration_ms;
    
    VPOperation operation = VPOperation::NONE;
};

class Arm_Solution{
    public:
        Arm_Motors_AngleClassis current_arm_motors_angle_ = {0.0f};
        bool PathOrganiser(PATH pathsign, const VPposition& path_start,
                           uint32_t segment, float progress, VPposition& output);
        Arm_Motors_AngleClassis getangles(void);
        VPposition calculateAnglestoVP(const Arm_Motors_AngleClassis &arm_motors_angle);
        // max_step_deg bounds every joint against the latest measured angle.
        IKResult calculateVPtoAngles(VPposition VP, float max_step_deg = 3600.0f);
        IKResult calculateVPtoAngles(VPposition VP,
            const Arm_Motors_AngleClassis& measured, float max_step_deg,
            float min_lift_deg = -1.0f);
        IKResult calculateVPtoAnglesAtLift(VPposition VP, float lift_deg);
        // Solve the circle through three points; fails when they are (near-)collinear.
        static bool circleFromThreePoints(const position& from, const position& via,
                                          const position& end, position& center);
        bool calculatePathlimit(const Arm_Motors_AngleClassis& start, const Arm_Motors_AngleClassis& target);
    private:
        float valueVP(const Arm_Motors_AngleClassis& current, const Arm_Motors_AngleClassis& target);
        bool isValid(const Arm_Motors_AngleClassis& angles, const VPposition& VP);
        VPposition VP_{0.0f, 0.0f, 0, 0.0f};
        position bottom{0.0f, 0.0f};
        Arm_Motors_AngleClassis arm_motors_angle_{0.0f};
};


struct PathSegment {
    position end;
    PathCurve curve;
    position center;    //画圆专用 
    bool counter_clockwise;
};

constexpr PathSegment start_path[] = {
    // Leave the home configuration vertically before moving toward the target.
    // {{131.0f + kArmXCalibrationOffsetMm, 1050.0f}, PathCurve::Line, {0.0f, 0.0f}, true},
    // Raise the rear of the arm above the forbidden zone before ROTATE moves.
    {{131.0f + kArmXCalibrationOffsetMm, 1300.0f}, PathCurve::Line, {0.0f, 0.0f}, true},
    // {{691.0f + kArmXCalibrationOffsetMm, 1200.0f}, PathCurve::Line, {0.0f, 0.0f}, true},
    {{691.0f + kArmXCalibrationOffsetMm, 1100.0f}, PathCurve::Line, {0.0f, 0.0f}, true},
    // {{691.0f + kArmXCalibrationOffsetMm, 750.0f}, PathCurve::Line, {0.0f, 0.0f}, true},
    // {{450.0f + kArmXCalibrationOffsetMm, 750.0f}, PathCurve::Line, {0.0f, 0.0f}, true},
};

// Trial 20 mm FETCH descent: physical results varied, so avoid a full 65 mm move.
constexpr float kFetchHeightTrimMm = 60.0f;
// FETCH bows toward positive X; its circle is fitted to the actual START endpoint.
constexpr float kFetchArcBulgeMm = 45.0f;
constexpr PathSegment fetch_path[] = {
    // At Y=520 mm and direction -180 deg, X≈706.7 mm approaches EXTEND=1200 deg
    // while retaining the final START lift branch.
    {{740.0f, 460.0f - kFetchHeightTrimMm},
     PathCurve::Arc, {0.0f, 0.0f}, false},
};

constexpr float kFetchEndDirectionDeg = -190.0f;
// FETCH height is now solved in the same calibrated coordinate frame as START.
// Apply the same descent to LIFT so ROTATE and EXTEND cannot cancel it.
constexpr float kFetchEndLiftDeg = 380.0f -
    kFetchHeightTrimMm / (kDISTANCE_2_DEG_withoutD * lift_D);
// constexpr float kFetchEndExtendDeg = 1200.0f;

// The HOLD arc passes above the vehicle before descending to its endpoint.
constexpr position kHoldArcVia{710.5f, 895.0f};
constexpr PathSegment  hold_path[] = {
    {{290.0f + kArmXCalibrationOffsetMm, 1050.0f}, PathCurve::Arc, {0.0f, 0.0f}, true},
};

// Legacy HOLD pose supplies the endpoint direction; joint angles are solved
// against the current geometry along the arc.
// constexpr Arm_Motors_AngleClassis kHoldEndAngles{
//     600.0f, -19.776423f, -90.0f, 618.23966f
// };

namespace arm_pathsequences
{
    inline const ArmPathStep start_pathsteps[] = {
        {PathCurve::Line, 131.0f + kArmXCalibrationOffsetMm, 1300.0f, -90.0f,
             0.0f, 0.0f, 0.0f, true,
              1000.0f, VPOperation::NONE},
        {PathCurve::Line, 691.0f + kArmXCalibrationOffsetMm, 1100.0f, -90.0f,
             -1199.0f, 0.0f, 0.0f, true,
              2500.0f, VPOperation::NONE},
        
    };

    inline const ArmPathStep fetch_pathsteps[] = {
        {PathCurve::Arc, 740.0f, 420.0f - kFetchHeightTrimMm, -185.0f,
             0.0f, 0.0f, 0.0f, false,
            5000.0f,VPOperation::FETCH_KFS},
    };

    inline const ArmPathStep holding_pathsteps[] = {
        {PathCurve::Arc, 290.0f + kArmXCalibrationOffsetMm, 1050.0f, -90.0f, 
            0.0f, 710.5f, 895.0f, true,
            5000.0f, VPOperation::FETCH_KFS},
    };
};
