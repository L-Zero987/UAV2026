#ifndef __LQR_HPP
#define __LQR_HPP


#ifdef __cplusplus
extern "C" {
#endif


/* Include */
#include "stm32f4xx.h"
#include "cmsis_os.h"
#include <string.h>
#include "main.h"


#ifdef __cplusplus
}




/* Define */
#ifndef lqr_abs
#define lqr_abs(x) ((x > 0) ? x : -x)
#endif

#ifndef user_malloc
#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif
#endif


namespace lqr_alg_n{

  typedef struct 
  {
    uint8_t System_State_Size=0;     //对应u与input
    uint8_t Control_Size=0;          //对应Output

    //非线性控制量
    float Control_Variable=0;
    float Control_Area=0;      //控制区域
    
    float *Input=nullptr;
    float *Output=nullptr;
    float *k=nullptr;           //最优反馈增益矩阵
    
    float *target=nullptr;
    
    void (*User_Func_f)(void);

  }lqr_alg_t;

  //lqr算法类
  class lqr_alg_c{
    public:
      lqr_alg_c();
      lqr_alg_c(uint8_t system_state_size, uint8_t control_size, float *k);
      void ECF_LQR_Init(uint8_t system_state_size, uint8_t control_size, float *k);
      void ECF_LQR_Data_Update(float* system_state);
      float ECF_LQR_Calculate(void);
      void ECF_LQR_Data_Clear(void);

    private:
      lqr_alg_t lqr_data_;
    
  };

}



#endif


#endif


