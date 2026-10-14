#include "stair_seq.hpp"
#include "point_tracker.hpp"
#include "Motor.hpp"
#include "FreeRTOS.h"
#include "task.h"
// 档位判据
constexpr uint16_t kPotLeftMax  = 600;     // pot < 此值   上楼模式
constexpr uint16_t kPotRightMin = 1400;    // pot > 此值   下楼模式

// 状态
constexpr uint8_t kPhaseIdle  = 0;
constexpr uint8_t kPhaseRun   = 1;
constexpr uint8_t kPhaseEstop = 2;
constexpr uint8_t kDirUp      = 0;
constexpr uint8_t kDirDown    = 1;

volatile uint8_t stair_phase = kPhaseIdle;     
volatile uint8_t stair_step  = 0;              
volatile uint8_t stair_dir   = kDirUp;         // 0=上楼 1=下楼

uint32_t stair_start_ms   = 0;                 // 本步开始时刻
float stair_start_dist = 0.0f;              // 本步开始时的里程 
bool stair_cmd_sent = false;
static float stair_odom_y_mm = 0.0f;
static float stair_lift_2006_dist_mm = 0.0f;
static bool stair_self_trigger = true; 

constexpr uint8_t kStepAction[5]     = { 0, 1, 2, 1, 0 };// 0=底盘按里程走  1=3508换目标等到位  2=2006按里程走
constexpr int8_t  kStepLiftTarget[5] = { 0, +1, 0, -1, 0 };// +1高位  -1低位  0不动
constexpr uint8_t kStepHighMode[5]   = { 0, 0, 1, 1, 0 };
constexpr float   kStepDistMM[2][5] = {
    {  +285.0f,  0.0f,  +631.0f,  0.0f,  +284.0f },   // 上楼流程中间每步距离
    {  -284.0f,  0.0f,  -631.0f,  0.0f,  -285.0f},   // 下楼
};
constexpr float   kStepSpeed[5]      = { 0.125f, 0.0f, 0.3f, 0.0f, 0.15f };// 每步速度，底盘用 m/s，2006用手柄归一化(-1~1)
constexpr uint32_t kStepTimeoutMs[5] = { 6000, 7000, 20000, 6000, 5000 };// 每步超时（ms）
constexpr uint8_t kActDrive = 0;     
constexpr uint8_t kActLift  = 1;      
constexpr uint8_t kActHigh  = 2; 

static void stairEnterStep(uint8_t step, uint32_t now_ms,
                           float odom_y_mm, float lift_2006_dist_mm)
{
    stair_start_ms = now_ms;
    if      (kStepAction[step] == kActDrive) stair_start_dist = odom_y_mm;
    else if (kStepAction[step] == kActHigh)  stair_start_dist = lift_2006_dist_mm;
    else                                     stair_start_dist = 0.0f;
    stair_cmd_sent = false;
}

 void stairSeq_Request(uint8_t dir) {
    stair_dir   = dir;
                stair_step  = 0;
                stair_phase = kPhaseRun;
                stairEnterStep(0, xTaskGetTickCount(), stair_odom_y_mm, stair_lift_2006_dist_mm);
}

bool stairSeq_Busy(void) {
            return stair_phase != kPhaseIdle;     
        }

void stairSeq_SetSelfTrigger(bool en) { stair_self_trigger = en; }

void stairSeq_Update(bool rm_updated,           
                     const pub_RM_Data &rc,
                     float odom_x_mm, 
                     float odom_y_mm,
                     float yaw_rel_deg,
                     float lift_2006_dist_mm,
                     bool  lift_at_target,
                     pub_lift_cmd    &lift_cmd,
                     pub_chassis_cmd &chassis_cmd,
                     PointTracker &tracker,
                     const pub_chassis_cmd &track_out 
                    )
{
    stair_odom_y_mm = odom_y_mm;
    stair_lift_2006_dist_mm = lift_2006_dist_mm;
    bool swE_edge = false;
    static bool swE_last = false;
    if (rm_updated) {
        const bool now = (rc.swE == RM_2_POS_SW_State_t::DOWN);
        swE_edge = now && !swE_last;
        swE_last = now;
    }

    if (stair_phase == kPhaseIdle)
    {
        if (swE_edge&& stair_self_trigger) {
            if (rc.pot < kPotLeftMax) {
                stair_dir   = kDirUp;
                stair_step  = 0;
                stair_phase = kPhaseRun;
                stairEnterStep(0, xTaskGetTickCount(), odom_y_mm, lift_2006_dist_mm);
            } else if (rc.pot > kPotRightMin) {
                stair_dir   = kDirDown;
                stair_step  = 0;
                stair_phase = kPhaseRun;
                stairEnterStep(0, xTaskGetTickCount(), odom_y_mm, lift_2006_dist_mm);
            }
        }
    }
    else if (stair_phase == kPhaseRun)
    {
        if (swE_edge) { stair_phase = kPhaseEstop; return;}
        const uint32_t now_ms = xTaskGetTickCount();
        const uint8_t  act    = kStepAction[stair_step];
        lift_cmd.hold               = false; 
        lift_cmd.high_mode          = (kStepHighMode[stair_step] != 0);
        chassis_cmd.yaw_hold_enable = (kStepHighMode[stair_step] == 0);
        chassis_cmd.turn_delta_deg  = 0.0f;      

        bool done = false;
        float moved = 0.0f;
       if (act == kActDrive)
        {
            if (!stair_cmd_sent)
            {
                const float th = yaw_rel_deg * DEGREE_2_RAD;
                const float D = kStepDistMM[stair_dir][stair_step];
                tracker.SetTarget(odom_x_mm, odom_y_mm, yaw_rel_deg);
                tracker.SetTarget(odom_x_mm - D * sinf(th),
                                  odom_y_mm + D * cosf(th), yaw_rel_deg);
            }
            float vx = track_out.linear_x_; 
            float vy = track_out.linear_y_; 
            const float vmax = kStepSpeed[stair_step];
            const float vmag = sqrtf(vx * vx + vy * vy);
            if (vmag > vmax)
            {
                const float k = vmax / vmag;
                vx *= k;
                vy *= k;
            }
            chassis_cmd.linear_x_    = vx;
            chassis_cmd.linear_y_    = vy;
            chassis_cmd.omega_ = 0.0f;
            lift_cmd.lift_2006_input = 0.0f;
            lift_cmd.request_high = false;
            lift_cmd.request_low = false;

            done = tracker.Arrived();
        }
        else if (act == kActHigh)
        {
            chassis_cmd.linear_x_    = 0.0f;
            chassis_cmd.linear_y_    = 0.0f;
            chassis_cmd.omega_       = 0.0f;
            lift_cmd.lift_2006_input = kStepSpeed[stair_step] * ((stair_dir == kDirUp) ? +1.0f : -1.0f);
            lift_cmd.request_high    = false;
            lift_cmd.request_low     = false;

            const float goal  = kStepDistMM[stair_dir][stair_step];
            moved = lift_2006_dist_mm - stair_start_dist;
            done = (fabsf(moved) >= fabsf(goal));
        }
        else if (act == kActLift)
        {
            chassis_cmd.linear_x_    = 0.0f;
            chassis_cmd.linear_y_    = 0.0f;
            chassis_cmd.omega_       = 0.0f;
            lift_cmd.lift_2006_input = 0.0f;
            lift_cmd.request_high    = (kStepLiftTarget[stair_step] > 0);
            lift_cmd.request_low     = (kStepLiftTarget[stair_step] < 0);

            done = lift_at_target;
        }
        if (!stair_cmd_sent) { stair_cmd_sent = true; done = false; }
        const bool timeout = (now_ms - stair_start_ms) >= kStepTimeoutMs[stair_step];
        if (done || timeout) {
            stair_step++;
            if (stair_step >= 5) { stair_step = 0; stair_phase = kPhaseIdle; }
            else stairEnterStep(stair_step, now_ms, odom_y_mm, lift_2006_dist_mm);
        }
    }
    else  
    {
    lift_cmd.hold       = true;                                 
    lift_cmd.request_high    = false;                               
    lift_cmd.request_low     = false;
    lift_cmd.lift_2006_input = 0.0f;
    lift_cmd.high_mode       = (kStepHighMode[stair_step] != 0);     

    chassis_cmd.linear_x_       = 0.0f;
    chassis_cmd.linear_y_       = 0.0f;
    chassis_cmd.omega_          = 0.0f;
    chassis_cmd.turn_delta_deg  = 0.0f;
    chassis_cmd.yaw_hold_enable = (kStepHighMode[stair_step] == 0);   
        if (swE_edge) {
            stair_start_ms = xTaskGetTickCount(); 
            stair_cmd_sent = false;  
            stair_phase = kPhaseRun;
        }
    }
}