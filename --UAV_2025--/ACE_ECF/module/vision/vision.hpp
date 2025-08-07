#ifndef __VISION_HPP
#define __VISION_HPP

#ifdef __cplusplus
extern "C"
{
#endif
#include "visual_config.h"
#include "fifo.h"
#include "usbd_cdc_if.h"


#ifdef __cplusplus
}
// #ifndef __VISUAL_CONFIG_H
// #define __VISUAL_CONFIG_H

class vision_c
{
private:
    /* data */
public:
    // Visual_Config_t config={0};
    Visual_Tx_t tx_data ={0};
    Visual_Rx_t rx_data={0};
    fifo_s_t *usb_fifo=nullptr;
    vision_c(/* args */);
    ~vision_c();
    uint8_t vision_send_data(float Pitch, float Yaw, float Roll);
    void vision_receive(void);
};
void vision_init();
vision_c *get_visiual_data();
bool get_vision_lost();
void vision_lost();
void change_color(uint8_t color_);
float change_pitch_compansate(float change);
#endif


#endif /*__VISION_HPP*/