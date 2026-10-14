#pragma once
#include <cstdint>
#include "topic_pool.h"
#include "point_tracker.hpp"

enum MissionAct : uint8_t { kActWalkTo = 0, kActClimb = 1, kActTurn = 2 };

struct MissionStep {
    uint8_t act;
    float   x, y;       
    uint8_t climb_dir;   // 0=上 1=下
    float   turn_deg;    // +左 −右
};

constexpr MissionStep kMission[] = {
    { kActWalkTo, -1600, 2095, 0, 0 },
    { kActClimb,      0,    0, 0, 0 },
    { kActClimb,      0,    0, 0, 0 },
    { kActClimb,      0,    0, 0, 0 },
    { kActTurn,       0,    0, 0, +180 },
    { kActClimb,      0,    0, 1, 0 },
    { kActTurn,       0,    0, 0, +90 },
    { kActClimb,      0,    0, 1, 0 },
    { kActClimb,      0,    0, 1, 0 },
};

constexpr uint8_t kMissionLen = sizeof(kMission) / sizeof(kMission[0]);

void mission_Update(bool rm_updated, const pub_RM_Data &rc,
                    float odom_x_mm, float odom_y_mm, float yaw_rel_deg,
                    PointTracker &tracker,
                    const pub_chassis_cmd &track_out,
                    pub_chassis_cmd &chassis_cmd);