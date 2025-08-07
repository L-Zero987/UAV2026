// 负责发送和解算接收视觉的数据
#include "vision.hpp"
#include "safe_task.hpp"
fifo_s_t *usb_info;
vision_c instance;
Safe_task_c *vision_safe = nullptr;
bool vision_is_lost = false; // 视觉丢失标志位
void vision_init()
{
    vision_safe = new Safe_task_c("vision", 200, vision_lost, nullptr);
    instance.usb_fifo = fifo_s_create(96); // 2倍数组大小
    usb_info = instance.usb_fifo;
    instance.rx_data.distance = -1;
    instance.tx_data.header = 0xFF;
    instance.tx_data.trailer = 0xFE;
}
vision_c::vision_c(/* args */)
{
}
void vision_lost()
{
    instance.rx_data.distance = -1;
    instance.rx_data.fire_flag = 0;
    vision_is_lost = true;
}
bool get_vision_lost()
{
    return vision_is_lost;
}
vision_c::~vision_c()
{
}
vision_c *get_visiual_data()
{
    return &instance;
}
// uint64_t send_time = 0; //发送次数
float Send_Pitch_Compensate = 1.0f; // 发送的pitch补偿
uint8_t color = 1;                  // 0-red 1-blue
uint8_t vision_c::vision_send_data(float Pitch, float Yaw, float Roll)
{
    // send_time++;
    static uint8_t send_buff[33];
    tx_data.header = 0xFF;
    tx_data.trailer = 0xFE;
    // 这里将memcpy 改为memset 可能会由问题

    memset(&tx_data.aim_x, 0, 12);
    tx_data.Roll = Roll;
    tx_data.Pitch = Pitch + Send_Pitch_Compensate;
    tx_data.Yaw = Yaw;
    tx_data.detect_color = color;
    tx_data.reset_tracker = 0;
    tx_data.reserved = 0;
    tx_data.bullet_speed = 24;
    memcpy(&send_buff, &tx_data.header, 33);
    if (CDC_Transmit_FS(send_buff, sizeof(Visual_Tx_t)) == USBD_OK)
    {
        return 1u;
    }
    return 0u;
}
void vision_c::vision_receive(void)
{
    uint8_t read_buff[sizeof(Visual_Rx_t)];
    fifo_s_gets(usb_fifo, (char *)read_buff, sizeof(Visual_Rx_t));
    if (read_buff[0] == 0xFF && read_buff[16] == 0xFE)
    {
        vision_safe->Online();
        memcpy(&rx_data, read_buff, sizeof(Visual_Rx_t));
    }
}
// 0-red 1-blue
void change_color(uint8_t color_) // 0-red 1-blue
{
    color = color_;
}
float change_pitch_compansate(float change)
{
    Send_Pitch_Compensate += change;
    return Send_Pitch_Compensate;
}
void safe_flesh()
{
    vision_safe->Online();
    vision_is_lost = false;
}