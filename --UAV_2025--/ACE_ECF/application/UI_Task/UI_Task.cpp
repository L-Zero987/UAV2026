#include "UI_Task.hpp"
#include "robot_cmd.hpp"
// 测试位置用
// float x1_1 = 239;
// float y1_1 = 413;
// float x2_2 = 353;
// float y2_2 = 473;
// uint8_t aim = 0;
// uint8_t fir = 0;
// uint8_t auto_ = 0;
//具体ui看比赛效果
namespace REFEREE_n
{
    namespace CILENT_UI_n
    {

        // UI_Draw_c::UI_Draw_c(UART_HandleTypeDef *huart_)

        /**
         * @brief  设置机器人ID
         */
        // void UI_Draw_c::Set_robo_id(uint16_t sender_id_)

        /**
         * @brief  绘制一个图形
         */
        // void UI_Draw_c::Draw_A_Graph(GraphicCommand *graphic_)
        UI_Draw_c *ui_instance = nullptr; // 全局指针
        void Cilent_UI_Init()
        {
            // static_ui_add(ui_instance);
            // dynamic_ui_ope(ui_instance, REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_ADD);
        }
        uint8_t cnt = 0;
        void Cilent_UI_Task()
        {
            vTaskDelay(1000);
            ui_instance = new UI_Draw_c(&UI_HUART);
            while (Get_referee_Address()->Robot_Status.robot_id == 0)
                ;
            ui_instance->Set_robo_id(Get_referee_Address()->Robot_Status.robot_id);
            static state_ui_c state_ui;
            static Crosshair crosshair;
            // static aware_c aware;
            crosshair.Set_Fire_Mode(0);
            crosshair.Set_Aim_Status(0);
            crosshair.Set_fric_status(0);

            //=====================================
            // auto arc1 = std::make_unique<REFEREE_n::CILENT_UI_n::ArcCommand>("999", CROSSHAIR_SIGHT_X, CROSSHAIR_SIGHT_Y, 100, 300, 100, REFEREE_n::CILENT_UI_n::Graph_Color_e::UI_Color_White, 2, 7);
            // while (1)
            // {
            //     crosshair.Set_Fire_Mode(1);
            //     crosshair.Set_Aim_Status(1);
            //     crosshair.Set_fric_status(1);
            //     arc1->configure_add();
            //     ui_instance->Draw_A_Graph(arc1.get());
            //     ui_instance->Draw_Group_Graph(crosshair.crosshair_init());
            // }
            //=====================================

            // ui_instance->Draw_A_Graph(state_ui.fir_string_init());
            // ui_instance->Draw_A_Graph(state_ui.auto_string_init());
            // ui_instance->Draw_A_Graph(state_ui.aim_string_init());
            // ui_instance->Draw_A_Graph(state_ui.Hz_string_init());
            // ui_instance->Draw_A_Graph(state_ui.Hz_state_init());
            ui_instance->Draw_Group_Graph(state_ui.all_state_init());

            ui_instance->Draw_Group_Graph(crosshair.crosshair_init());

            // ui_instance->Draw_A_Graph(state_ui.pitch_string_init());
            // ui_instance->Draw_A_Graph(state_ui.rpm_string_init());
            gimbal_control_t *gimbal_cmd = ROBOT_CMD_N::get_gimbal_data();
            shoot_mood_t *shoot_cmd = ROBOT_CMD_N::get_shoot_data();
            REFEREE_t *pReferee = Get_referee_Address();

            while (1)
            {
             
                // 更新状态
                state_ui.set_pitch_comp(change_pitch_compansate(0));
                state_ui.set_rpm_speed((6050 + shoot_cmd->firction_compansate));
                state_ui.set_fir_state(shoot_cmd->friction_mode);
                state_ui.set_aim_state(gimbal_cmd->vision_is_on == AUTO_AIM_MODE); //
                state_ui.set_auto_state(!get_vision_lost());
                state_ui.set_hz_state(shoot_cmd->fire_hz);
                crosshair.Set_fric_status((uint8_t)shoot_cmd->friction_mode);
                crosshair.Set_Aim_Status((uint8_t)gimbal_cmd->vision_is_on); // 自瞄状态 0：未开启自瞄，1：开启自瞄但是没瞄上，2：开启自瞄且瞄上了
                crosshair.Set_Fire_Mode(shoot_cmd->shoot_num == 1);
                // aware.change_aware(pReferee->Game_Status.stage_remain_time);
                if (cnt++ % 10 == 0)
                {
                    ui_instance->Draw_Group_Graph(crosshair.crosshair_init());
                    ui_instance->Draw_A_Graph(state_ui.pitch_string_init());
                    ui_instance->Draw_A_Graph(state_ui.rpm_string_init());

                    // ui_instance->Draw_A_Graph(state_ui.Hz_string_init());
                    state_ui.set_hz_state(shoot_cmd->fire_hz);
                    ui_instance->Draw_A_Graph(state_ui.Hz_state_init());
                    ui_instance->Draw_A_Graph(state_ui.fir_string_init());
                    ui_instance->Draw_A_Graph(state_ui.auto_string_init());
                    ui_instance->Draw_A_Graph(state_ui.aim_string_init());
                    ui_instance->Draw_Group_Graph(state_ui.all_state_init());
                //   ui_instance->Draw_Group_Graph(aware._state_init());
                    cnt = 1;
                }
                else
                {
                    // ui_instance->Draw_Group_Graph(aware._state_update());
                    ui_instance->Draw_Group_Graph(state_ui.all_state_update());
                    ui_instance->Draw_A_Graph(state_ui.Hz_state_update());
                    ui_instance->Draw_Group_Graph(crosshair.crosshair_update());
                }
            }
        }

    }
}