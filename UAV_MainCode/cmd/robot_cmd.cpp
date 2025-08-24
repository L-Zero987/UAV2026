#include "robot_cmd.hpp"
#include <bitset>

namespace RobotCMD_n
{
/* region 宏定义与全局变量 */
    std::bitset<256> key_state;     // 按键状态
    RobotCMD_c *this_ptr;           // 本类实例指针
// endregion

/* region 实例创建 */
    RobotCMD_c *RobotCMD_c::Get_InstancePtr() {
        static RobotCMD_c _instance;
        this_ptr = &_instance;
        return &_instance;
    }

    RobotCMD_c::RobotCMD_c() {
        this->RC_instance_p = ECF_RC::getInstance();
        this->DR16_cmd = &this->RC_instance_p->Dt7;         // 不要合并的数据
        this->RE_date = this->RC_instance_p->getREdata();
        this->TC_cmd = &this->RE_date->PHOTO_ctrl;
        ECFRC_Init();
    }
// endregion

/* region 功能函数 */
    RobotCMD_ConnectState_e RobotCMD_c::Get_ConnectState()
    {
        return this->connect_state;
    }

    // region DR16相关
    RobotCMD_ValueState_e RobotCMD_c::Get_RC_SW1State()
    {
        switch (this->DR16_cmd->rc.s1)
        {
        case RC_SW_UP:
            return CMD_HIGH;
        case RC_SW_MID:
            return CMD_MIDDLE;
        case RC_SW_DOWN:
            return CMD_LOW;
        default:
            return CMD_ERROR;
        }
    }

    RobotCMD_ValueState_e RobotCMD_c::Get_RC_SW2State()
    {
        switch (this->DR16_cmd->rc.s2)
        {
            case RC_SW_UP:
                return CMD_HIGH;
            case RC_SW_MID:
                return CMD_MIDDLE;
            case RC_SW_DOWN:
                return CMD_LOW;
            default:
                return CMD_ERROR;
        }
    }

    int16_t RobotCMD_c::Get_RC_RatchetValue()
    {
        return this->DR16_cmd->rc.ch[4];
    }

    int16_t RobotCMD_c::Get_RC_RJoyLRValue()
    {
        return this->DR16_cmd->rc.ch[0];
    }

    int16_t RobotCMD_c::Get_RC_RJoyUDValue()
    {
        return this->DR16_cmd->rc.ch[1];
    }

    int16_t RobotCMD_c::Get_RC_LJoyLRValue()
    {
        return this->DR16_cmd->rc.ch[2];
    }

    int16_t RobotCMD_c::Get_RC_LJoyUDValue()
    {
        return this->DR16_cmd->rc.ch[3];
    }

    RobotCMD_ValueState_e RobotCMD_c::Get_RC_RatchetState()
    {
        return this->ratchet_state;
    }
    // endregion

    // region TC相关
    int16_t RobotCMD_c::Get_TC_MouseXValue()
    {
        return this->TC_cmd->mouse.x;
    }

    int16_t RobotCMD_c::Get_TC_MouseYValue()
    {
        return this->TC_cmd->mouse.y;
    }

    int16_t RobotCMD_c::Get_TC_MouseZValue()
    {
        return this->TC_cmd->mouse.z;
    }

    /**
     * @brief 检测按键函数
     * @attention 三种按键检测函数共享按键状态缓存，一个按键不能被多个目标检测
     */
    bool RobotCMD_c::Check_TC_KeyDown(char key_name, uint8_t key_value)
    {// (O,o)? 这个强转会产生-127 = 127的情况吗?
        if(key_value && !key_state.test(static_cast<size_t>(key_name))) // 按键按下 且 上次记录为 未按下
        {
            key_state.set(static_cast<size_t>(key_name));// 记录按键按下
            return true; // 返回是
        }
        else if(key_value) // 按键按下 且 上次记录为 按下
        {
            return false; // 返回否
        }
        key_state.reset(static_cast<size_t>(key_name)); // 按键未按下，记录未按下
        return false; // 返回否
    }

    bool RobotCMD_c::Check_TC_KeyUp(char key_name, uint8_t key_value)
    {
        if(key_value) // 按钮按下
        {
            key_state.set(static_cast<size_t>(key_name)); // 记录按钮按下
            return false;  // 返回否
        }
        else if(key_state.test(static_cast<size_t>(key_name))) // 按钮未按下，若上次记录为 按下
        {
            key_state.reset(static_cast<size_t>(key_name)); // 记录按钮未按下
            return true;   // 返回是
        }
        return false;   // 返回否
    }

    bool RobotCMD_c::Check_TC_KeyPress(char key_name, uint8_t key_value)
    {
        if(key_value)key_state.set(static_cast<size_t>(key_name));
        else key_state.reset(static_cast<size_t>(key_name));
        return key_value;
    }

    void RobotCMD_c::Change_RatchetState()
    {
        // 将棘轮的模拟信号转化为状态
        static int8_t _tolerance_value[6] = {0};       // 容差值，防止在临界值附近频繁切换状态
        static RobotCMD_ValueState_e _last_state = CMD_MIDDLE;

        memset(_tolerance_value, 0, 6 * sizeof(_tolerance_value[0]));
        _tolerance_value[_last_state] = 10;

        if (this->DR16_cmd->rc.ch[4] > (-20 + _tolerance_value[CMD_MIDDLE_LOW]) &&
            this->DR16_cmd->rc.ch[4] < (20 - _tolerance_value[CMD_MIDDLE_HIGH]))
        {
            _last_state = CMD_MIDDLE;
            this->ratchet_state = CMD_MIDDLE;
        }
        else if (this->DR16_cmd->rc.ch[4] > (20 + _tolerance_value[CMD_MIDDLE]) &&
                 this->DR16_cmd->rc.ch[4] < (640 - _tolerance_value[CMD_HIGH]))
        {
            _last_state = CMD_MIDDLE_HIGH;
            this->ratchet_state = CMD_MIDDLE_HIGH;
        }
        else if (this->DR16_cmd->rc.ch[4] > (640 + _tolerance_value[CMD_MIDDLE_HIGH]))
        {
            _last_state = CMD_HIGH;
            this->ratchet_state = CMD_HIGH;
        }
        else if (this->DR16_cmd->rc.ch[4] < (-20 - _tolerance_value[CMD_MIDDLE]) &&
                 this->DR16_cmd->rc.ch[4] > (-640 + _tolerance_value[CMD_LOW]))
        {
            _last_state = CMD_MIDDLE_LOW;
            this->ratchet_state = CMD_MIDDLE_LOW;
        }
        else if (this->DR16_cmd->rc.ch[4] < (-640 - _tolerance_value[CMD_MIDDLE_LOW]))
        {
            _last_state = CMD_LOW;
            this->ratchet_state = CMD_LOW;
        }
    }
    // endregion

// endregion

/* region 状态机 */
    void StateLoop()
    {
        switch (this_ptr->connect_state)
        {
            case NOT_CONNECT:
                if (this_ptr->RC_instance_p->error_flag.bit.photo)
                {
                    this_ptr->connect_state = TC_CMD;
                    return;
                }
                else if (this_ptr->RC_instance_p->error_flag.bit.dr16)
                {
                    this_ptr->connect_state = DR16_CMD;
                    return;
                }
                break;
            case DR16_CMD:
                if (this_ptr->RC_instance_p->error_flag.bit.photo)
                {
                    this_ptr->connect_state = TC_CMD;
                    return;
                }
                else if (!this_ptr->RC_instance_p->error_flag.bit.dr16)
                {
                    this_ptr->connect_state = NOT_CONNECT;
                    return;
                }
                this_ptr->Change_RatchetState();
                break;
            case TC_CMD:
                if (!this_ptr->RC_instance_p->error_flag.bit.photo)
                {
                    key_state.reset();
                    if (this_ptr->RC_instance_p->error_flag.bit.dr16)
                    {
                        this_ptr->connect_state = DR16_CMD;
                        return;
                    }
                    else
                    {
                        this_ptr->connect_state = NOT_CONNECT;
                        return;
                    }
                }
                break;
            default:
                break;
        }
    }
// endregion
}
