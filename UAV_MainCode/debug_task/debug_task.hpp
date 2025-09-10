#ifndef __DEBUG_TASK_HPP
#define __DEBUG_TASK_HPP

#include "fire.hpp"
#include "gimbal.hpp"

namespace Debug_n
{
    class Debug_c
    {
    public:
        Fire_n::Fire_c *fire_ptr = nullptr;
        Gimbal_n::Gimbal_c *gimbal_ptr = nullptr;
        Debug_c(void);

        void MainLoop(void);
        void MotorLoop(void);
        void StateLoop(void);
    private:
    };

}



#endif //! __DEBUG_TASK_HPP
