#include "debug_task.hpp"

namespace Debug_n
{
    Debug_c::Debug_c(void)
    {
        this->fire_ptr = Fire_n::Fire_c::Get_InstancePtr();
//        this->gimbal_ptr = Gimbal_n::Gimbal_c::Get_InstancePtr();
    }

    void Debug_c::MainLoop()
    {

    }

    void Debug_c::MotorLoop()
    {

    }

    void Debug_c::StateLoop()
    {

    }
}
