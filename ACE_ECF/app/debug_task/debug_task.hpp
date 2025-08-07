#ifndef __DEBUG_TASK_HPP
#define __DEBUG_TASK_HPP

#include "DJI_motor.hpp"

namespace Debug_n
{
    typedef struct
    {

    } Debug_Log_t;

    typedef struct
    {

    } Debug_Value_t;

    class Debug_c
    {
    public:

        Debug_c(void);

        void Loop1(void);
    private:
        DJI_Motor_n::DJI_Motor_Instance *test_motor;
    };

}



#endif //! __DEBUG_TASK_HPP
