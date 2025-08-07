#ifndef __VISUAL_CONFIG_H
#define __VISUAL_CONFIG_H

#include "fifo.h"
#pragma pack(1)
typedef struct
{
    uint8_t header; //帧头
    uint8_t detect_color;  // 颜色  0-red 1-blue
    uint8_t reset_tracker; // 重置
    uint8_t reserved; //保留位
    float bullet_speed; //弹速
    float Roll; //当前云台姿态
    float Pitch; 
    float Yaw;
    float aim_x;
    float aim_y;
    float aim_z;
    uint8_t trailer; //帧尾
} Visual_Tx_t; //发送的信息

typedef struct
{
    uint8_t header; 
    uint8_t fire_flag; //是否开火
    float pitch;  //
    float yaw;      
    float distance;  //距离 为-1时丢失目标
    uint8_t mode;
    uint8_t armor_nums; // 装甲板数字
    uint8_t trailer; 
} Visual_Rx_t;
#pragma pack()
typedef struct 
{
    Visual_Tx_t tx_data;
    Visual_Rx_t rx_data;
    fifo_s_t *usb_fifo;
} Visual_Config_t;

void safe_flesh();
#endif
