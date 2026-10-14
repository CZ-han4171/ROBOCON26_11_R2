#include "mission.hpp"
#include "stair_seq.hpp"
#include "tool.hpp"
#include "Motor.hpp"
#include <cmath>
#include "FreeRTOS.h"
#include "task.h"

constexpr uint8_t kMissionIdle = 0, kMissionRun = 1, kMissionEstop = 2;

volatile uint8_t mission_phase = kMissionIdle;
volatile uint8_t mission_index = 0;
static bool  mission_unit_entered = false;   
static float turn_target_deg = 0.0f;        

constexpr float kWalkSpeedMax = 0.3f;        
constexpr float kTurnTolDeg   = 1.0f;       

static void missionEnterUnit(uint8_t idx)
{
    mission_unit_entered = false;           
}

void mission_Update(bool rm_updated, const pub_RM_Data &rc,
                    float odom_x_mm, float odom_y_mm, float yaw_rel_deg,
                    PointTracker &tracker,
                    const pub_chassis_cmd &track_out,
                    pub_chassis_cmd &chassis_cmd)
{
    bool swE_edge = false;
    static bool swE_last = false;
    if (rm_updated) {
        const bool now = (rc.swE == RM_2_POS_SW_State_t::DOWN);
        swE_edge = now && !swE_last;
        swE_last = now;
    }

    if (mission_phase == kMissionIdle)
    {
        if (swE_edge) {
            mission_index = 0;
            mission_phase = kMissionRun;
            missionEnterUnit(0);
            stairSeq_SetSelfTrigger(false);
        }
        return;                            
    }

    if (mission_phase == kMissionEstop)
    {
        chassis_cmd.linear_x_       = 0.0f;
        chassis_cmd.linear_y_       = 0.0f;
        chassis_cmd.omega_          = 0.0f;
        chassis_cmd.turn_delta_deg  = 0.0f;
        if (swE_edge) { mission_phase = kMissionRun; }
        return;
    }

    if (swE_edge) { mission_phase = kMissionEstop; return; }
      // 以下即为 RUN 状态
    const MissionStep &s = kMission[mission_index];
    bool done = false;

   if (s.act == kActWalkTo)
    {
        if (!mission_unit_entered) {                 
            tracker.SetTarget(odom_x_mm, odom_y_mm, yaw_rel_deg);   
            tracker.SetTarget(s.x, s.y, yaw_rel_deg);             
        }
        float vx = track_out.linear_x_;
        float vy = track_out.linear_y_;
        const float v = sqrtf(vx * vx + vy * vy);     
        if (v > kWalkSpeedMax) { vx *= kWalkSpeedMax / v; vy *= kWalkSpeedMax / v; }
        chassis_cmd.linear_x_      = vx;
        chassis_cmd.linear_y_      = vy;
        chassis_cmd.omega_         = 0.0f;
        chassis_cmd.turn_delta_deg = 0.0f;
        done = tracker.Arrived();
    }
    else if (s.act == kActClimb)
    {
        if (!mission_unit_entered) { stairSeq_Request(s.climb_dir); }
        done = !stairSeq_Busy();
    }
    else 
    {
        if (!mission_unit_entered) {
            turn_target_deg = normalizeDeg(yaw_rel_deg + s.turn_deg);
            chassis_cmd.turn_delta_deg = s.turn_deg;    
        }
        chassis_cmd.linear_x_ = 0.0f;                   
        chassis_cmd.linear_y_ = 0.0f;
        done = fabsf(normalizeDeg(turn_target_deg - yaw_rel_deg)) < kTurnTolDeg;
    }

    if (!mission_unit_entered) { mission_unit_entered = true; done = false; }
                                                           

    if (done) {
        mission_index++;
        if (mission_index >= kMissionLen) {
            mission_index = 0;
            mission_phase = kMissionIdle;
            stairSeq_SetSelfTrigger(true);
        } else {
            missionEnterUnit(mission_index);
        }
    }
}