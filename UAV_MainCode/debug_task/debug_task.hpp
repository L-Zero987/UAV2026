#ifndef __DEBUG_TASK_HPP
#define __DEBUG_TASK_HPP

#include "DJI_motor.hpp"
#include "fire.hpp"

namespace Debug_n
{
    class Debug_c
    {
    public:

        Debug_c(void);

        void MainLoop(void);
        void MotorLoop(void);
        void StateLoop(void);
    private:
    };

}



#endif //! __DEBUG_TASK_HPP
