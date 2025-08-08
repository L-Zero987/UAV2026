#include "debug_task.hpp"

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
        fire->friction_motor[0]->DJIMotorSetRef(1000);
        fire->friction_motor[1]->DJIMotorSetRef(1000);
        fire->friction_motor[2]->DJIMotorSetRef(1000);
    }
}
