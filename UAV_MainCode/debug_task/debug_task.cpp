#include "debug_task.hpp"

float speed1;
float speed2;
float speed3;
float set_speed = -1000;

namespace Debug_n
{
    Debug_c::Debug_c(void)
    {
        this->fire = Fire_n::Fire_c::Get_InstancePtr();
        fire->friction_motor[0]->DJIMotorEnable();
        fire->friction_motor[1]->DJIMotorEnable();
        fire->friction_motor[2]->DJIMotorEnable();
    }

    void Debug_c::Loop1(void)
    {
        fire->friction_motor[0]->DJIMotorEnable();
        fire->friction_motor[1]->DJIMotorEnable();
        fire->friction_motor[2]->DJIMotorEnable();
        fire->friction_motor[0]->DJIMotorSetRef(set_speed);
        fire->friction_motor[1]->DJIMotorSetRef(set_speed);
        fire->friction_motor[2]->DJIMotorSetRef(set_speed);
    }
}
