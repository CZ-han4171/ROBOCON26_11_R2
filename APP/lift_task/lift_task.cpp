#include "lift_task.h"

#include "Motor.hpp"
#include "pid_controller.h"
#include "topic_pool.h"
#include "topics.hpp"
#include "tool.hpp"

extern C610Motor lift_2006_motor1;
extern C610Motor lift_2006_motor2;
extern C620Motor lift_3508_motor1;
extern C620Motor lift_3508_motor2;

osThreadId_t LiftTaskHandle; 

static TypedTopicSubscriber<pub_lift_cmd> lift_cmd_sub("lift_cmd", 8);
pub_lift_cmd lift_cmd_recv{};

extern volatile float g_chassis_yaw_deg;
extern volatile float yaw_lock_deg;
extern volatile bool imu_ready;
static float high_yaw_lock_ref = 0.0f;
static bool  high_yaw_ready = false;

constexpr float LIFT_LOW_POS_DEG  =  +60.0f;   // 缩回离地（低位）
constexpr float LIFT_HIGH_POS_DEG = -485.0f;   // 伸出顶车（高位）
constexpr float LIFT_MAX_SPEED_REF = 5.0f;  
constexpr float LIFT_MAX_SYNC_COMP = 1.5f; 
constexpr float LIFT_2006_MOTOR1_DIR = -1.0f;
constexpr float LIFT_2006_MOTOR2_DIR = +1.0f;
constexpr float MAX_LIFT_2006_SPEED = 16.0f;

float current_lift_2006_speed1 = 0.0f;
float current_lift_2006_speed2 = 0.0f;
float current_lift_3508_motor1_pos = 0.0f;
float current_lift_3508_motor2_pos = 0.0f;
float current_lift_3508_motor1_speed = 0.0f;
float current_lift_3508_motor2_speed = 0.0f;
float target_angle = 0.0f;
float lift_3508_avg_pos = 0.0f;
float lift_3508_diff_pos = 0.0f;
float lift_3508_err_deg = 0.0f;
volatile bool lift_at_target = false;
volatile float lift_2006_dist_mm = 0.0f;  
static float   lift_2006_last1 = 0.0f;    
static float   lift_2006_last2 = 0.0f;     
static bool    lift_2006_odom_ready = false;
static bool lift_hold_prev = false;
float ds1 = 0.0f;
float ds2 = 0.0f;

float LIFT_2006_MM_PER_DEG =  0.349f;

float lift_2006_motor1_pid_out = 0.0f;
float lift_2006_motor2_pid_out = 0.0f;
float lift_3508_motor1_pid_out = 0.0f;
float lift_3508_motor2_pid_out = 0.0f;

PID_t lift_sync_pid = {
    .Kp = 8.6f,             
    .Ki = 0.2f,
    .Kd = 0.0f,
    .MaxOut = LIFT_MAX_SYNC_COMP,   
    .IntegralLimit = 1.5f,
    .DeadBand = 0.5f * DEGREE_2_RAD,             
    .Improve = Integral_Limit,
};

PID_t lift_pos_pid = {
    .Kp = 5.7f,          
    .Ki = 0.3f,
    .Kd = 0.0f,
    .MaxOut = LIFT_MAX_SPEED_REF,      
    .IntegralLimit = 1.0f,
    .DeadBand = 0.0f,
    .Improve = Integral_Limit,
};
PID_t lift_3508_motor1_pid = {
    .Kp = 3500.0f,
    .Ki = 7600.0f,
    .Kd = 0.0f,
    .MaxOut = 16000.0f,
    .DeadBand = 0.0f,
    .Improve = IMCREATEMENT_OF_OUT,
};

PID_t lift_3508_motor2_pid = {
    .Kp = 3500.0f,
    .Ki = 7600.0f,
    .Kd = 0.0f,
    .MaxOut = 16000.0f,
    .DeadBand = 0.0f,
    .Improve = IMCREATEMENT_OF_OUT,
};

PID_t lift_2006_motor1_pid = {
    .Kp = 1500.0f,            
    .Ki = 3000.0f,
    .Kd = 0.0f,
    .MaxOut = 10000.0f,       
    .DeadBand = 0.0f,        
    .Improve = IMCREATEMENT_OF_OUT,
};

PID_t lift_2006_motor2_pid = {
    .Kp = 1500.0f,            
    .Ki = 3000.0f,
    .Kd = 0.0f,
    .MaxOut = 10000.0f,       
    .DeadBand = 0.0f,        
    .Improve = IMCREATEMENT_OF_OUT,
};

PID_t high_yaw_lock_pid = {
    .Kp = 5.7f,                       
    .Ki = 0.2f,
    .Kd = 0.0f,
    .MaxOut = 2.0f,                   
    .IntegralLimit = 1.0f,
    .DeadBand = 0.5f * DEGREE_2_RAD,  
    .Improve = Integral_Limit,
};

static void lift_3508_step() {
    current_lift_3508_motor1_pos = lift_3508_motor1.getCurrentSumPos();
    current_lift_3508_motor2_pos = lift_3508_motor2.getCurrentSumPos();
    current_lift_3508_motor1_speed = lift_3508_motor1.getCurrentSpeed();
    current_lift_3508_motor2_speed = lift_3508_motor2.getCurrentSpeed();
    current_lift_2006_speed1 = lift_2006_motor1.getCurrentSpeed();
    current_lift_2006_speed2 = lift_2006_motor2.getCurrentSpeed();
   
    lift_3508_avg_pos  = (current_lift_3508_motor1_pos + current_lift_3508_motor2_pos) * 0.5f; //平均位置
    lift_3508_diff_pos = current_lift_3508_motor1_pos - current_lift_3508_motor2_pos;    //位置差值
    lift_3508_err_deg = target_angle - lift_3508_avg_pos;
    lift_at_target = (fabsf(lift_3508_err_deg) < 2.0f) && (fabsf(current_lift_3508_motor1_speed) < 0.5f) && (fabsf(current_lift_3508_motor2_speed) < 0.5f);
    float avg_rad = lift_3508_avg_pos * DEGREE_2_RAD;
    float tgt_rad = target_angle * DEGREE_2_RAD;
    float speed_cmd = PID_Calculate(&lift_pos_pid, avg_rad, tgt_rad);
    //同步环
    float diff_rad = lift_3508_diff_pos * DEGREE_2_RAD;
    float sync_cmd = PID_Calculate(&lift_sync_pid, diff_rad, 0.0f);
    float ref_3508_m1 = speed_cmd + sync_cmd;
    float ref_3508_m2 = speed_cmd - sync_cmd;
    if (ref_3508_m1 >  LIFT_MAX_SPEED_REF) ref_3508_m1 =  LIFT_MAX_SPEED_REF;   
    if (ref_3508_m1 < -LIFT_MAX_SPEED_REF) ref_3508_m1 = -LIFT_MAX_SPEED_REF;
    if (ref_3508_m2 >  LIFT_MAX_SPEED_REF) ref_3508_m2 =  LIFT_MAX_SPEED_REF;
    if (ref_3508_m2 < -LIFT_MAX_SPEED_REF) ref_3508_m2 = -LIFT_MAX_SPEED_REF; 
    // 内层速度环（rad/s）
    lift_3508_motor1_pid_out = PID_Calculate(&lift_3508_motor1_pid, current_lift_3508_motor1_speed, ref_3508_m1);
    lift_3508_motor2_pid_out = PID_Calculate(&lift_3508_motor2_pid, current_lift_3508_motor2_speed, ref_3508_m2);
    lift_3508_motor1.setMotorCmd(lift_3508_motor1_pid_out);
    lift_3508_motor2.setMotorCmd(lift_3508_motor2_pid_out);
    // 2006速度环（rad/s）
    float speed_2006 = 0.0f;
    float omega_lock = 0.0f;

    if (lift_cmd_recv.high_mode && imu_ready) {
        speed_2006 = lift_cmd_recv.lift_2006_input * MAX_LIFT_2006_SPEED;
        const float lock_err_rad =
            normalizeDeg(high_yaw_lock_ref - g_chassis_yaw_deg) * DEGREE_2_RAD;
        omega_lock = PID_Calculate(&high_yaw_lock_pid, 0.0f, lock_err_rad);
    }
    const float now1 = lift_2006_motor1.getCurrentSumPos();
    const float now2 = lift_2006_motor2.getCurrentSumPos();
    if (!lift_2006_odom_ready) {
        lift_2006_last1 = now1;
        lift_2006_last2 = now2;
        lift_2006_odom_ready = true;
    } else {
        ds1 = (now1 - lift_2006_last1) * LIFT_2006_MM_PER_DEG;
        ds2 = (now2 - lift_2006_last2) * LIFT_2006_MM_PER_DEG;
        lift_2006_dist_mm += (ds1 * LIFT_2006_MOTOR1_DIR + ds2 * LIFT_2006_MOTOR2_DIR) / 2.0f;
        lift_2006_last1 = now1;
        lift_2006_last2 = now2;
    }
    const float ref_2006_m1 = (speed_2006 + omega_lock) * LIFT_2006_MOTOR1_DIR;
    const float ref_2006_m2 = (speed_2006 - omega_lock) * LIFT_2006_MOTOR2_DIR;

    lift_2006_motor1_pid_out = PID_Calculate(&lift_2006_motor1_pid,
                                             current_lift_2006_speed1, ref_2006_m1);
    lift_2006_motor2_pid_out = PID_Calculate(&lift_2006_motor2_pid,
                                             current_lift_2006_speed2, ref_2006_m2);
    lift_2006_motor1.setMotorCmd(lift_2006_motor1_pid_out);
    lift_2006_motor2.setMotorCmd(lift_2006_motor2_pid_out);
}

void liftInit(void) {
    PID_Init(&lift_3508_motor1_pid);
    PID_Init(&lift_3508_motor2_pid);
    PID_Init(&lift_pos_pid);
    PID_Init(&lift_sync_pid);
    PID_Init(&lift_2006_motor1_pid);
    PID_Init(&lift_2006_motor2_pid);
    target_angle = LIFT_LOW_POS_DEG; 
}  
void liftTask(void *argument) {
    (void)argument;
    TickType_t currentTime = xTaskGetTickCount();
    liftInit();
    for (;;) {
   if (lift_cmd_sub.TryGet(&lift_cmd_recv)) {
         if (!lift_cmd_recv.hold) {
        if (lift_cmd_recv.request_high) {
            target_angle = LIFT_HIGH_POS_DEG;
        } else if (lift_cmd_recv.request_low) {
            target_angle = LIFT_LOW_POS_DEG;
        }
    }
    }                                 
    if (lift_cmd_recv.hold) {          
    if (lift_hold_prev == false) {    
        target_angle = lift_3508_avg_pos;  
        lift_pos_pid.Iout  = 0.0f;        
        lift_pos_pid.ITerm = 0.0f;        
    }                                 
    lift_hold_prev = true;           
} else {                            
    lift_hold_prev = false;          
}  
    if (lift_cmd_recv.high_mode) {
        if (!high_yaw_ready && imu_ready) {
            high_yaw_lock_ref = yaw_lock_deg;
            PID_Init(&high_yaw_lock_pid);
            high_yaw_ready = true;
        }
    } else {
        high_yaw_ready = false;     
    }
        lift_3508_step();
        vTaskDelayUntil(&currentTime, 1);
    }
}