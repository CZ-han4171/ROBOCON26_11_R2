#pragma once

#include "topic_pool.h"
#include "point_tracker.hpp"

void stairSeq_Request(uint8_t dir);   // 0=上 1=下，请求启动一次爬楼
bool stairSeq_Busy(void); 
void stairSeq_SetSelfTrigger(bool en);
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
                    );