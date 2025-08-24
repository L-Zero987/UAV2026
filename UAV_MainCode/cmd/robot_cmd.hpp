#ifndef __ROBOT_CMD_HPP
#define __ROBOT_CMD_HPP

#include "RC.hpp"

#define CMD_HIGH_VALUE        660
#define CMD_MIDDLE_HIGH_VALUE 330
#define CMD_MIDDLE_VALUE      0
#define CMD_MIDDLE_LOW_VALUE  -330
#define CMD_LOW_VALUE         -660

namespace RobotCMD_n
{
    typedef enum
    {
        CMD_ERROR       =0x00u,
        CMD_LOW         =0x01u,
        CMD_MIDDLE_LOW  =0x02u,
        CMD_MIDDLE      =0x03u,
        CMD_MIDDLE_HIGH =0x04u,
        CMD_HIGH        =0x05u
    }RobotCMD_ValueState_e;

    typedef enum
    {
        NOT_CONNECT = 0x00u,
        DR16_CMD    = 0x01u,
        TC_CMD      = 0x02u
    }RobotCMD_ConnectState_e;

    class RobotCMD_c
    {
    public:
        ECF_RC* RC_instance_p;
        RC_ctrl_t*    DR16_cmd;
        PHOTO_ctrl_t* TC_cmd;
        REFEREE_t*    RE_date;

        RobotCMD_ValueState_e ratchet_state = CMD_MIDDLE;
        RobotCMD_ConnectState_e connect_state = NOT_CONNECT;

        static RobotCMD_c* Get_InstancePtr(void);

        RobotCMD_ConnectState_e Get_ConnectState(void);
        RobotCMD_ValueState_e Get_RC_SW1State(void);
        RobotCMD_ValueState_e Get_RC_SW2State(void);
        int16_t Get_RC_RatchetValue(void);
        int16_t Get_RC_RJoyLRValue(void);
        int16_t Get_RC_RJoyUDValue(void);
        int16_t Get_RC_LJoyLRValue(void);
        int16_t Get_RC_LJoyUDValue(void);
        RobotCMD_ValueState_e Get_RC_RatchetState(void);
        int16_t Get_TC_MouseXValue(void);
        int16_t Get_TC_MouseYValue(void);
        int16_t Get_TC_MouseZValue(void);
        bool Check_TC_KeyDown(char key_name, uint8_t key_value);
        bool Check_TC_KeyUp(char key_name, uint8_t key_value);
        bool Check_TC_KeyPress(char key_name, uint8_t key_value);

        friend void StateLoop(void);

    private:
        RobotCMD_c();

        void Change_RatchetState(void);
    };
    void StateLoop(void);
}

#endif //! __ROBOT_CMD_HPP
