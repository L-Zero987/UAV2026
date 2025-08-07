#ifndef __ROBOT_CMD_HPP
#define __ROBOT_CMD_HPP

#ifdef __cplusplus
extern "C"
{
#endif


#ifdef __cplusplus
}
#endif
typedef enum
{
    Gimbal_TC,
    Gimbal_RC
} Control_State_e; // 实际操控源
#define TC_CONTROL 1
#include "dr16.hpp"
#include "robot_def.hpp"
namespace ROBOT_CMD_N
{
    class robot_cmd_c
    {
    private:
        /* data */
    public:
        robot_cmd_c(/* args */);
        ~robot_cmd_c();
        void handle_s1_up(uint8_t s2_state);
        void handle_s1_mid(uint8_t s2_state);
        void handle_s1_down(uint8_t s2_state);
        void RC_fire_command();
        void vision_command();
        void Get_TC_data();
        void gimbal_change_ecd();
        DR16_n::RC_ctrl_t *RC_HandleData = nullptr;
        gimbal_control_t gimbal_cmd = {.mode = GIMBAL_ZERO_FORCE};
        shoot_mood_t shoot_cmd = {.friction_mode = FRICTION_OFF, .loader_mode = LOADER_OFF};
        void remote_control_set(void);
        void keyboard_control();
        Control_State_e control_type = Gimbal_RC;//默认是遥控器控制
    };

    gimbal_control_t *get_gimbal_data();
    shoot_mood_t *get_shoot_data();
    void robot_cmd_init(void);
    void robot_cmd_task(void);
    // void robot_cmd_task(Visual_Rx_t &visual_rx);
}

#endif /*__ROBOT_CMD_HPP*/