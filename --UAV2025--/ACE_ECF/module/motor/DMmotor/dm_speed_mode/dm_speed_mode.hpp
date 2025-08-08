#ifndef __DM_SPEED_MODE_HPP
#define __DM_SPEED_MODE_HPP

#include "dm_motor.hpp"

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#ifdef __cplusplus
}
#endif

namespace DM_Motor_n
{
    class DM_Speed_Mode_c:public DM_Motor_Main_c,public BSP_CAN_Part_n::CANInstance_c
    {
        private:
        void TxDataProcessing(float _vel); 
        public:
        DM_Speed_Mode_c(CAN_HandleTypeDef *can_handle,
                      DM_ModePrame_s *param_, 
                      uint32_t tx_id, 
                      uint32_t rx_id,
                      uint32_t SAND_IDE
                      );
        void Transmit(float _vel);
        void StartStateSet(DM_StartState_e State);
    };
    
}   





#endif /*__DM_SPEED_MODE_HPP*/