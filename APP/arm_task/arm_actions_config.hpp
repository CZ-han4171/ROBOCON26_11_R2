#pragma once

#include <cstdint>
#include <cstddef>
#include "arm_solution.hpp"

struct motor_pose{
    float pos = 0.0f;
    float speed = 0.0f;
};

inline constexpr VPOperation operator|(VPOperation a, VPOperation b)
{
    return static_cast<VPOperation>(static_cast<uint8_t>(a) | static_cast<uint8_t>(b));
}

inline constexpr uint8_t operator&(VPOperation a, VPOperation b)
{
    return static_cast<uint8_t>(a) & static_cast<uint8_t>(b);
}

struct arm_pose{
    uint32_t wait_ms{0};
    motor_pose lift;
    motor_pose rotate;
    motor_pose flip;
    motor_pose extend;
    VPOperation operation = VPOperation::NONE;
};

struct pose_seq{
    const arm_pose* data;
    uint32_t size;
    template<std::size_t N>
    constexpr pose_seq(const arm_pose (&arr)[N]) : data(arr), size(static_cast<uint32_t>(N)) {}
};

// namespace arm_sequences
// {
//     inline const arm_pose start_pose[] = {
//         { 10, {570.0f, 1000.0f}, {90.0f, 120.0f}, {-85.0f, 2.3f}, {630.0f, 20.0f} , VPOperation::NONE},
//         { 600, {600.0f, 1000.0f}, {0.0f, 120.0f}, {0.0f, 2.3f}, {560.0f, 20.0f} , VPOperation::NONE},
//         { 500, {}, {}, {}, {}, VPOperation::SKIP_MOTORS | VPOperation::RESET},
//     };

//     inline const arm_pose reset_pose = {0, {570.0f, 1000.0f}, {0.0f, 120.0f}, {0.0f, 3.0f}, {0.0f, 18.0f} , VPOperation::NONE};


//     // 吸取 +200 台阶 KFS
//     inline const arm_pose fetch_step_M[] = {  // height, flip, rotate, expand
//         { 10, {520.0f, 1000.0f}, {82.0f, 120.0f}, {4.0f, 2.2f}, {1200.0f, 20.0f} },
//         { 980, {365.0f, 1000.0f}, {82.0f, 120.0f}, {21.5f, 2.1f}, {1200.0f, 20.0f}, VPOperation::FETCH_KFS},
//         { 440, {660.0f, 1000.0f}, {82.0f, 120.0f}, {0.0f, 2.1f}, {600.0f, 18.0f} },
//         { 300, {570.0f, 1000.0f}, {-90.0f, 100.0f}, {0.0f, 2.8f}, {0.0f, 20.0f} },
        
//     };
//     // 吸取 +400 台阶 KFS（存了0个）
//     inline const arm_pose fetch_step_H[] = {
//         { 10, {600.0f, 1000.0f}, {90.0f, 120.0f}, {-11.0f, 2.2f}, {1200.0f, 21.0f} },
//         { 1230, {590.0f, 1000.0f}, {90.0f, 120.0f}, {3.0f, 2.1f}, {1200.0f, 18.0f}, VPOperation::FETCH_KFS },
//         { 340, {920.0f, 1000.0f}, {88.0f, 120.0f}, {-16.0f, 2.0f}, {800.0f, 18.0f} },
//         { 480, {570.0f, 1000.0f}, {-90.0f, 90.0f}, {0.0f, 2.8f}, {0.0f, 21.0f} },  // 加速处理

//     };
//     // 吸取 -200 台阶 KFS（存了0个）
//     inline const arm_pose fetch_step_L[] = {
//         { 10, {-240.0f, 1000.0f}, {56.5f, 120.0f}, {33.6f, 2.1f}, {1200.0f, 18.0f}, VPOperation::FETCH_KFS },
//         { 2380, {660.0f, 1000.0f}, {80.0f, 120.0f}, {33.6f, 2.1f}, {800.0f, 18.0f} },
//         { 280, {570.0f, 1000.0f}, {-5.0f, 40.0f}, {0.0f, 1.9f}, {730.0f, 17.0f} },
//         { 1200, {570.0f, 1000.0f}, {-90.0f, 56.0f}, {0.0f, 2.0f}, {0.0f, 17.0f} },

//     };
//     // 吸取 +0 台阶 KFS（存了0个）
//     inline const arm_pose fetch_step_P[] = {
//         { 10, {500.0f, 1000.0f}, {50.0f, 120.0f}, {22.0f, 2.2f}, {1170.0f, 20.0f} },
//         { 600, {300.0f, 1000.0f}, {50.0f, 120.0f}, {40.0f, 2.1f}, {1170.0f, 18.0f}, VPOperation::FETCH_KFS },
//         { 700, {920.0f, 1000.0f}, {50.0f, 80.0f}, {22.0f, 2.0f}, {880.0f, 18.0f} },
//         { 900, {570.0f, 1000.0f}, {-90.0f, 70.0f}, {0.0f, 2.2f}, {0.0f, 18.0f} },

//     };
//     inline const pose_seq fetch[4] = {fetch_step_L, fetch_step_P, fetch_step_M, fetch_step_H };

//     inline const arm_pose place_2_kfs_1[] = {
//         { 10, {80.0f, 1000.0f}, {-78.0f, 120.0f}, {-192.0f, 6.8f}, {790.0f, 21.0f}, VPOperation::FETCH_KFS },
//         { 2450, {1040.0f, 1000.0f}, {-106.0f, 90.0f}, {-160.0f, 2.7f}, {710.0f, 21.0f} },
//         { 1200, {1020.0f, 1000.0f}, {-81.5f, 100.0f}, {-19.0f, 3.1f}, {660.0f, 18.0f} },
//         { 1250, {1020.0f, 1000.0f}, {6.0f, 90.0f}, {-19.0f, 3.1f}, {660.0f, 18.0f} },
    
//     };
//     // 取出KFS（存了2个）
//     inline const arm_pose place_2_kfs_2[] = {
//         { 10, {1040.0f, 1000.0f}, {-90.0f, 120.0f}, {0.0f, 3.1f}, {880.0f, 28.0f} },
//         { 700, {900.0f, 1000.0f}, {-88.0f, 120.0f}, {-200.0f, 6.8f}, {720.0f, 28.0f}, VPOperation::FETCH_KFS },
//         { 2200, {1020.0f, 1000.0f}, {-81.5f, 100.0f}, {-19.0f, 3.1f}, {660.0f, 18.0f} },
//         { 1400, {1020.0f, 1000.0f}, {6.0f, 90.0f}, {-19.0f, 3.1f}, {660.0f, 18.0f} },
//     };
//     // 取出KFS（存了3个）
//     inline const arm_pose place_2_kfs_3[] = {
//         { 10, {1020.0f, 1000.0f}, {6.5f, 120.0f}, {-22.0f, 3.1f}, {660.0f, 18.0f} }
//     };
    

//     // 取出KFS（存了1个）（放置第三层）
//     inline const arm_pose place_3_kfs_1[] = {
//         { 10, {80.0f, 1000.0f}, {-78.0f, 120.0f}, {-192.0f, 6.8f}, {790.0f, 21.0f}, VPOperation::FETCH_KFS },
//         { 2450, {1040.0f, 1000.0f}, {-106.0f, 90.0f}, {-160.0f, 2.7f}, {710.0f, 21.0f} },
//         { 1200, {1020.0f, 1000.0f}, {-81.5f, 100.0f}, {-28.7f, 3.1f}, {660.0f, 18.0f} },
//         { 1100, {1020.0f, 1000.0f}, {15.0f, 110.0f}, {-28.7f, 3.1f}, {660.0f, 18.0f} },

//     };
//     // 取出KFS（存了2个）（放置第三层）
//     inline const arm_pose place_3_kfs_2[] = {
//         { 10, {1040.0f, 1000.0f}, {-90.0f, 120.0f}, {0.0f, 3.1f}, {880.0f, 28.0f} },
//         { 700, {900.0f, 1000.0f}, {-88.0f, 120.0f}, {-200.0f, 6.8f}, {720.0f, 28.0f}, VPOperation::FETCH_KFS },
//         { 2200, {1020.0f, 1000.0f}, {-81.5f, 100.0f}, {-28.7f, 3.1f}, {660.0f, 18.0f} },
//         { 1400, {1020.0f, 1000.0f}, {15.0f, 110.0f}, {-28.7f, 3.1f}, {660.0f, 18.0f} },
//     };
//     // 取出KFS（存了3个）（放置第三层）
//     inline const arm_pose place_3_kfs_3[] = {
//         { 10, {1020.0f, 1000.0f}, {15.4f, 110.0f}, {-28.7f, 3.1f}, {660.0f, 18.0f} }
//     };
//     inline const pose_seq place[2][3] = { 
//         { place_2_kfs_1, place_2_kfs_2, place_2_kfs_3 }, 
//         { place_3_kfs_1, place_3_kfs_2, place_3_kfs_3 } 
//     };

//     inline const arm_pose place_release_proceed[] = {
//         { 10, {}, {}, {}, {}, VPOperation::SKIP_MOTORS | VPOperation::RELEASE_KFS },
//         { 300, {570.0f, 1000.0f}, {0.0f, 120.0f}, {12.0f, 2.1f}, {880.0f, 18.0f} },
//         { 500, {}, {}, {}, {}, VPOperation::SKIP_MOTORS | VPOperation::RESET },

//     };
//     inline const pose_seq release = place_release_proceed;

//     // 将 KFS 存入（当前剩0个KFS）
//     inline const arm_pose kfs_0[] = {
//         { 10, {680.0f, 1000.0f}, {-90.0f, 120.0f}, {-180.0f, 6.8f}, {460.0f, 18.0f} },
//         { 800, {500.0f, 1000.0f}, {-90.0f, 120.0f}, {-180.0f, 2.1f}, {420.0f, 18.0f} },
//         { 700, {}, {}, {}, {},VPOperation::SKIP_MOTORS | VPOperation::RELEASE_KFS },
//         { 200, {570.0f, 1000.0f}, {0.0f, 120.0f}, {0.0f, 2.2f}, {500.0f, 18.0f} },
//         { 1500, {}, {}, {}, {}, VPOperation::SKIP_MOTORS | VPOperation::RESET }
//     };
//     // 将 KFS 存入（当前剩1个KFS）
//     inline const arm_pose kfs_1[] = {
//         { 10, {880.0f, 1000.0f}, {-92.0f, 120.0f}, {-180.0f, 6.8f}, {429.5f, 18.0f} },
//         { 1650, {}, {}, {}, {}, VPOperation::SKIP_MOTORS | VPOperation::RELEASE_KFS },
//         { 200, {670.0f, 1000.0f}, {0.0f, 120.0f}, {10.0f, 2.2f}, {780.0f, 18.0f} },
//         { 1500, {}, {}, {}, {}, VPOperation::SKIP_MOTORS | VPOperation::RESET }
//     };
//     inline const pose_seq store[2] = { kfs_0, kfs_1 };
    
//     // 将 KFS 丢掉（当前剩1或2个KFS）
//     inline const arm_pose kfs_12[] = {
//         { 10, {720.0f, 1000.0f}, {0.0f, 120.0f}, {-150.0f, 6.8f}, {1200.0f, 18.0f} },
//         { 950, {}, {}, {}, {}, VPOperation::SKIP_MOTORS | VPOperation::RELEASE_KFS },
//         { 100, {}, {}, {}, {}, VPOperation::SKIP_MOTORS | VPOperation::RESET }
//     };
//     // 将 KFS 丢掉（当前剩3个KFS）
//     inline const arm_pose kfs_3[] = {
//         { 10, {720.0f, 1000.0f}, {-60.0f, 120.0f}, {0.0f, 6.8f}, {1200.0f, 18.0f} },
//         { 500, {720.0f, 1000.0f}, {-60.0f, 120.0f}, {-150.0f, 6.8f}, {1200.0f, 18.0f} },
//         { 950, {}, {}, {}, {}, VPOperation::SKIP_MOTORS | VPOperation::RELEASE_KFS },
//         { 380, {720.0f, 1000.0f}, {-60.0f, 120.0f}, {0.0f, 6.8f}, {1200.0f, 18.0f} },
//         { 1000, {}, {}, {}, {}, VPOperation::SKIP_MOTORS | VPOperation::RESET }
//     };
//     inline const pose_seq drop[2] = { kfs_12, kfs_3 };

// };

// void ConfigShifter()
