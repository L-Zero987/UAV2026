#include "main_task.hpp"
#include "debug_task.hpp"
#include "Config.hpp"

extern "C"
{
#include "FreeRTOS.h"
#include "task.h"
}

#if (IS_DEBUG_MODE == 0)

#else

 Debug_n::Debug_c* debug_instance;

#endif

void MainTask_Init()
{
#if (IS_DEBUG_MODE == 0)


#else
    debug_instance = new Debug_n::Debug_c();

#endif
}

void MainLoop_Task(void const * argument)
{
    while (1)
    {
#if (IS_DEBUG_MODE == 0)

#else
        debug_instance->Loop1();
#endif
        vTaskDelay(1);
    }
}

