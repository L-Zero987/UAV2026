#include "robot_cmd.hpp"
#include "gimbal.hpp"
/*
 *     开云台+自瞄
 *    ↗                   ↗ 棘轮向下拨一点时--打一梭子
 * 左上角 → 开云台      右上角  →棘轮向下拨一点时时单发
 *    ↘                   ↘
 *     无力                 最高权限失能摩擦轮
 *  拨轮只有在摩擦轮开启时才能开启 --》向上拨一点开摩擦轮
 *  向下拨一点：由右上拨杆决定
 *  向下波到底：连发
 * 图传（优先级高）：
 *  长按鼠标右键 自瞄
 *  鼠标左键 开拨弹盘
F：开关摩擦轮
C：加弹速RPM         ctrl+c：减弹速
V：加自瞄pitch补偿  ctrl+V：减补偿
G：高弹频               ctrl+G：低弹频
左键射弹，右键自瞄
 *
 */

namespace ROBOT_CMD_N
{
    static robot_cmd_c pRobot_cmd; // (O,o)? 为什么p
    robot_cmd_c::robot_cmd_c(/* args */)
    {
    }

    robot_cmd_c::~robot_cmd_c()
    {
    }
    float last_speed = 0; //记录上一次的弹速
    void robot_cmd_c::remote_control_set()
    {
        // 左1右2
        const uint8_t s1_state = RC_HandleData->rc.s1;
        const uint8_t s2_state = RC_HandleData->rc.s2;
        static REFEREE_t *referee_ptr = Get_referee_Address();
        static DR16_n::DR16_c *pDR16 = DR16_n::DR16_c::dr16_->GetClassPtr();
        if (referee_ptr != nullptr && !get_referee_lost()) //检测裁判系统是否存在
        {
            static uint8_t color = ((referee_ptr->Robot_Status.robot_id <= 11) ? 1 : 0); //读裁判系统判断颜色
            // static uint8_t color = 1; //对面蓝色为1，红色为0，比赛写死不出问题
            change_color(color);

            if (referee_ptr->Shoot_Data.bullet_speed > LIMIT_SPEED && shoot_cmd.firction_compansate > -700 && last_speed != referee_ptr->Shoot_Data.bullet_speed)
            {
                    shoot_cmd.firction_compansate -= 50; //防超射速锁发射机构
            }
            last_speed = referee_ptr->Shoot_Data.bullet_speed;
        }
        if (control_type == Gimbal_TC) // 以及图传是否活着
        {
            if (!get_TC_lost())
            {
                memcpy(&RC_HandleData->mouse.x, &referee_ptr->Remote_control, 10); // 搬运图传数据到遥控器结构体
                keyboard_control();
                return;
            }
            else if (pDR16->dt7_state_==DR16_n::LOST) 
            {
                gimbal_cmd.mode = GIMBAL_ZERO_FORCE;
                shoot_cmd.shoot_mode = SHOOT_OFF;
                shoot_cmd.friction_mode = FRICTION_OFF;
                shoot_cmd.loader_mode = LOADER_OFF;
                return;
            }
        }

        switch (s1_state)
        {
        case RC_SW_UP:
            // s1向上：开自瞄+云台使能
            handle_s1_up(s2_state);
            vision_command();
            // keyboard_control();
            /* code */
            break;
        case RC_SW_MID:
            ////s1居中:云台使能
            handle_s1_mid(s2_state);
            // keyboard_control();
            /* code */
            break;
        case RC_SW_DOWN:
            //  以s1为主逻辑向下：失能
            handle_s1_down(s2_state);
            return; // 该逻辑不需要视觉
            /* code */
            break;
        default:
            gimbal_cmd.mode = GIMBAL_ZERO_FORCE;
            shoot_cmd.shoot_mode = SHOOT_OFF;
            shoot_cmd.friction_mode = FRICTION_OFF;
            shoot_cmd.loader_mode = LOADER_OFF;
            break;
        }
    }

    void robot_cmd_c::handle_s1_up(uint8_t s2_state)
    {
        shoot_cmd.shoot_mode = SHOOT_ON;
        gimbal_cmd.mode = GIMBAL_ENABLE;
        gimbal_cmd.use_ecd = false;
        switch (s2_state)
        {
        case RC_SW_UP:
        case RC_SW_MID:
            RC_fire_command();
            break;
        case RC_SW_DOWN:
            shoot_cmd.friction_mode = FRICTION_OFF;
            shoot_cmd.loader_mode = LOADER_OFF;
            break;
        default:
            break;
        }
        gimbal_cmd.pitch_imu_target += RC_HandleData->rc.ch[1] * PITCH_SENSOR_RC;
        gimbal_cmd.yaw_imu_target -= RC_HandleData->rc.ch[0] * YAW_SENSOR_RC;
    }
    void robot_cmd_c::handle_s1_mid(uint8_t s2_state)
    {
        gimbal_cmd.vision_is_on = AIM_MANAUL_MODE; // 设为手瞄，取消自瞄
        gimbal_cmd.mode = GIMBAL_ENABLE;
        shoot_cmd.shoot_mode = SHOOT_ON;
        switch (s2_state)
        {
        case RC_SW_UP:
        case RC_SW_MID:
            gimbal_change_ecd();
            RC_fire_command();
            break;
        case RC_SW_DOWN:
            shoot_cmd.friction_mode = FRICTION_OFF;
            shoot_cmd.loader_mode = LOADER_OFF;
            break;
        default:
            break;
        }
        if(gimbal_cmd.use_ecd)
        {
            gimbal_cmd.pitch_ecd_target += RC_HandleData->rc.ch[1] * PITCH_SENSOR_RC;
            gimbal_cmd.yaw_ecd_target += RC_HandleData->rc.ch[0] * YAW_SENSOR_RC;
        }
        gimbal_cmd.pitch_imu_target += RC_HandleData->rc.ch[1] * PITCH_SENSOR_RC;
        gimbal_cmd.yaw_imu_target -= RC_HandleData->rc.ch[0] * YAW_SENSOR_RC;
    }
    void robot_cmd_c::handle_s1_down(uint8_t s2_state)
    {
        gimbal_cmd.vision_is_on = AIM_MANAUL_MODE;
        // gimbal_cmd.use_ecd = false;
        gimbal_cmd.use_ecd = true;
        gimbal_cmd.mode = GIMBAL_ZERO_FORCE;
        gimbal_cmd.pitch_ecd_target = 0;
        gimbal_cmd.yaw_ecd_target = 0;
        // gimbal_cmd.pitch_imu_target = -35 // 预防使能肘击
        shoot_cmd.shoot_mode = SHOOT_OFF;
        shoot_cmd.friction_mode = FRICTION_OFF;
        shoot_cmd.loader_mode = LOADER_OFF;
    }
    void robot_cmd_c::vision_command()
    {
        // test ：这样的写法测试是否会防止数据突变引起不稳定
        const Visual_Rx_t visual_rx = get_visiual_data()->rx_data;
        if (visual_rx.distance > 0) // 自瞄模式 且已识别到目标
        {
            gimbal_cmd.vision_is_on = AUTO_AIM_MODE;
            gimbal_cmd.yaw_imu_target = visual_rx.yaw; // pitch需要改成映射到达妙数据的形式
            gimbal_cmd.pitch_imu_target = visual_rx.pitch;
            uint8_t vision_on_fire = 0;
            if ((visual_rx.mode == 1) || (visual_rx.mode == 2 && visual_rx.fire_flag == 1))
            {
                vision_on_fire = 1;
            }
            else {
                vision_on_fire = 0;
            }
            if(shoot_cmd.friction_mode == FRICTION_ON)
            {
                if ((!GIMBAL_N::gimbal_c::get_instance()->fire_check()) || !vision_on_fire)
                {
                    shoot_cmd.loader_mode = LOADER_OFF;
                }
            }
            return;
        }
        else
        {
            gimbal_cmd.vision_is_on = AUTO_BUT_NOT_AIM;
        }
    }

    static uint32_t ecd_last_time = 0; //
    static uint32_t dead = 2000;
     void robot_cmd_c::gimbal_change_ecd()
     {
         if ((RC_HandleData->rc.ch[2] > 600 && RC_HandleData->rc.ch[3] < -600))
         {
             if (ecd_last_time + dead < xTaskGetTickCount())
             {
                 gimbal_cmd.use_ecd = true; // 使能编码器
                 gimbal_cmd.vision_is_on = AIM_MANAUL_MODE;
             }
         }
         else
         {
             ecd_last_time = xTaskGetTickCount();
         }
    }
    uint32_t fire_dead_time = 1500; //

    void robot_cmd_c::RC_fire_command()
    {
        constexpr int FIRE_ON_POS_THRESHOLD = 100;
        constexpr int FIRE_ON_SPEED_THRESHOLD = 550;
        constexpr int FRICTIOON_CHANGE_HIGH_THRESHOLD = -180;
        constexpr int FRICTIOON_CHANGE_LOW_THRESHOLD = -480;
        constexpr int RETURN_LOADER_THRESHOLD = -600;
        static uint32_t semi_last_time = 0;  // 单发时间
        const uint32_t semi_dead_time = 200; // 单发死区
        static uint32_t fire_last_time = 0;  //
        if ((RC_HandleData->rc.ch[4] > FRICTIOON_CHANGE_LOW_THRESHOLD && RC_HandleData->rc.ch[4] < FRICTIOON_CHANGE_HIGH_THRESHOLD) 
        && ((fire_last_time + fire_dead_time) < xTaskGetTickCount()))
        {
            fire_last_time = xTaskGetTickCount(); 
            // 拨轮向上拨一点 / 鼠标右键-- >,开关摩擦轮
            shoot_cmd.friction_mode = (friction_mode_e)!shoot_cmd.friction_mode; //  摩擦轮开关
        }
        if (shoot_cmd.friction_mode == FRICTION_ON) //
        {
            if ((RC_HandleData->rc.ch[4] > FIRE_ON_POS_THRESHOLD && RC_HandleData->rc.ch[4] < FIRE_ON_SPEED_THRESHOLD))
            {
                semi_last_time = xTaskGetTickCount(); // 记录单发时间
                shoot_cmd.loader_mode = LOADER_ON;
                shoot_cmd.shoot_num = RC_HandleData->rc.s2 == RC_SW_UP ? 8 : 1; // 右侧拨杆向上连发8发，否则单发
                shoot_cmd.fire_hz = RC_HandleData->rc.s2 == RC_SW_UP?1:0;
            }
            else if ((RC_HandleData->rc.ch[4] > FIRE_ON_SPEED_THRESHOLD + 50) && ((semi_last_time + semi_dead_time) < xTaskGetTickCount()))
            {
                // 单发触发后，给定死区防止连发
                shoot_cmd.loader_mode = LOADER_ON;
                shoot_cmd.shoot_num = 0;
            }
            else if (RC_HandleData->rc.ch[4] < RETURN_LOADER_THRESHOLD) // 拨到底
            {
                shoot_cmd.loader_mode = LOADER_ON_REVERSE;
            }
            else
            {
                shoot_cmd.loader_mode = LOADER_OFF;
            }
        }
        else
        {
            shoot_cmd.loader_mode = LOADER_OFF;
        }
    }
    
    void robot_cmd_c::keyboard_control()
    {
      static uint8_t last_F = 0;
      static uint8_t last_C = 0;
      static uint8_t last_V = 0;
      static uint8_t last_G = 0;
      static uint8_t last_R = 0;
      static uint8_t first_in = 0;
      shoot_cmd.shoot_mode = SHOOT_ON;
      gimbal_cmd.mode = GIMBAL_ENABLE;
      gimbal_cmd.use_ecd = false;
      gimbal_cmd.pitch_imu_target += RC_HandleData->mouse.y * PITCH_SENSOR_TC;
      gimbal_cmd.yaw_imu_target -= RC_HandleData->mouse.x * YAW_SENSOR_TC;
      if (RC_HandleData->mouse.press_r == 1)
      {
          vision_command();
      }
      else
      {
          gimbal_cmd.vision_is_on = AIM_MANAUL_MODE;
      }
        // if(RC_HandleData->kb.bit.E)
        // {
        //     gimbal_cmd.mode = GIMBAL_ENABLE;
        // }
        if(!first_in) //第一次进入图传直接开摩擦轮
        {
            // shoot_cmd.friction_mode = FRICTION_ON;
            shoot_cmd.shoot_num = 0;
            first_in = 1;
        }
        if (RC_HandleData->kb.bit.F == 1 && last_F == 0) // 关摩擦轮
        {
            shoot_cmd.friction_mode = (friction_mode_e)(!shoot_cmd.friction_mode);
        }
        if (shoot_cmd.friction_mode == FRICTION_ON) // 鼠标左键打开拨弹盘
        {
            shoot_cmd.loader_mode = (RC_HandleData->mouse.press_l == 1) ? LOADER_ON : LOADER_OFF; // 开拨弹盘
        }
        else
        {
            shoot_cmd.loader_mode = LOADER_OFF;
        }
        if (RC_HandleData->kb.bit.C == 1 && last_C == 0) // 加减弹速 有ctrl减
        {
            shoot_cmd.firction_compansate += (RC_HandleData->kb.bit.CTRL == 1) ? -50 : 50;
        }
        if (RC_HandleData->kb.bit.V == 1 && last_V == 0) // 加减picth补偿
        {
            change_pitch_compansate((RC_HandleData->kb.bit.CTRL == 1) ? -0.2 : 0.2);
        }
        if (RC_HandleData->kb.bit.G == 1 && last_G == 0) // 处理弹频调整
        {
            shoot_cmd.fire_hz = (RC_HandleData->kb.bit.CTRL == 1) ? 0 : 1;
        }
        if (RC_HandleData->kb.bit.R == 1 && last_R == 0) // 处理单发和连发
        {
            shoot_cmd.shoot_num = (RC_HandleData->kb.bit.CTRL == 1) ? 1 : 0;
        }
        last_F = RC_HandleData->kb.bit.F;
        last_C = RC_HandleData->kb.bit.C;
        last_V = RC_HandleData->kb.bit.V;
        last_G = RC_HandleData->kb.bit.G;
        last_R = RC_HandleData->kb.bit.R;

    }
    // bool fire_vision_cheak()
    // {
        
    // }
    void robot_cmd_c::Get_TC_data()
    {
        // static uint8_t last_flag = 0;
        // if(TCreceive_Flag != last_flag)
        // {
        //     memcpy(&RC->mouse.x, &TC_data->Remote_control, 10);//搬运图传数据到遥控器结构体
        // }
        // last_flag = TCreceive_Flag;
    }
    /*外部调用的函数*/
    void robot_cmd_init(void)
    {
        DR16_n::DR16_c *pDR16;
        pDR16 = DR16_n::DR16_c::dr16_->GetClassPtr();
        pDR16->dr16_rece_->USART_rx_start();
        pRobot_cmd.RC_HandleData = pDR16->GetDataStructPtr();
        pRobot_cmd.gimbal_cmd.pitch_imu_target = -35; //防肘击
#ifdef TC_CONTROL
        pRobot_cmd.control_type = Gimbal_TC;
#endif // DEBUG
    }
    void robot_cmd_task(void)
    {
        pRobot_cmd.remote_control_set();
    }
    gimbal_control_t *get_gimbal_data()
    {
        return &(pRobot_cmd.gimbal_cmd);
    }
    shoot_mood_t *get_shoot_data()
    {
        return &(pRobot_cmd.shoot_cmd);
    }

}
