#include "arm_djmotor.hpp"

osThreadId_t ArmDJMotorTaskHandle{};

extern C610Motor arm2006_motor;
extern C620Motor arm3508_motor;

namespace{

    const Arm_DJMotor::PidParam kRotatePosPid(
        2.0f, 1.1f, 0.0f,
        0.01f, 60.0f, 0.001745f, NONE
    );

    const Arm_DJMotor::PidParam kRotateSpeedPid(
        15000.0f, 30000.0f, 0.0f,
        0.0f, 10000.0f, 0.00f, IMCREATEMENT_OF_OUT
    );

    const Arm_DJMotor::PidParam kExtendPosPid(
        2.0f, 2.0f, 0.0f,
        0.01f, 60.0f, 0.001745f, NONE
    );

    const Arm_DJMotor::PidParam kExtendSpeedPid(
        2500.0f, 12000.0f, 0.0f,
        1000.0f, 20000.0f, 0.0f, IMCREATEMENT_OF_OUT
    );

    auto &motor = arm3508_motor;
}

Arm_DJMotor arm_rotate_motor(
    arm3508_motor,
    kRotatePosPid,
    kRotateSpeedPid
);

Arm_DJMotor arm_extend_motor(
    arm2006_motor,
    kExtendPosPid,
    kExtendSpeedPid
);

void ArmDJMotorTask(void *argument)
{
    (void)argument;

    for (;;)
    {   
        if(arm2006_motor.isOffline() || arm3508_motor.isOffline())
        {
            arm2006_motor.setMotorCmd(0);
            arm3508_motor.setMotorCmd(0);
        }
        else
        {
            arm_rotate_motor.update();
            arm_extend_motor.update();
        }
        osDelay(1);
    }
}
