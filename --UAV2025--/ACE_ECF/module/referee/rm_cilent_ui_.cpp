#include "rm_cilent_ui_.hpp"

user_maths_c ui_math;


namespace REFEREE_n{
namespace CILENT_UI_n{

// UI_Draw_c::UI_Draw_c(UART_HandleTypeDef *huart_)

/**
 * @brief  设置机器人ID
 */
// void UI_Draw_c::Set_robo_id(uint16_t sender_id_)

/**
 * @brief  绘制一个图形
 */
// void UI_Draw_c::Draw_A_Graph(GraphicCommand *graphic_)

// const char fric[] = {"FRIC:"};
// const char pit[] = {"PITCH:"};
// const char rpm[] = {"RPM:"};
// const char Hz[] = {"FIRE_HZ:"};
// const static char *ON_OFF_STATE[2] = {
//     "OFF !", // 关闭摩擦轮
//     "ON !",  // 打开摩擦轮
// };
// const static char *fire_hz_state[2] = {
//     "LOW !",  //
//     "HIGH !", //
// };
// UI_Draw_c *ui_instance = nullptr; //全局指针
// void Cilent_UI_Init()
// {
//     ui_instance = new UI_Draw_c(&UI_HUART);
//     ui_instance->Set_robo_id(Get_referee_Address()->Robot_Status.robot_id);
//     static_ui_add(ui_instance);
//     dynamic_ui_ope(ui_instance, REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_ADD);
// }

// void Cilent_UI_Task()
// {
//     uint32_t cnt = 0;
//     while (1)
//     {
//         if (cnt % 10 == 0)
//         {
//             // dynamic_ui_ope(ui_instance, REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_Change);
//         }
//         cnt++;
//         vTaskDelay(30);
//     }
// }

// void static_ui_add(UI_Draw_c *ins)
// {

//     static auto fir_str = std::make_unique<REFEREE_n::CILENT_UI_n::string_data_c>("fir", fric, 808, 695, Graph_Color_e::UI_Color_Yellow, 1, 3, 30);
//     fir_str->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_ADD);
//     ins->Draw_A_Graph(fir_str.get());

//     static auto pitch_str = std::make_unique<REFEREE_n::CILENT_UI_n::string_data_c>("pit", pit, 130, 815, Graph_Color_e::UI_Color_Yellow, 1, 3, 30);
//     pitch_str->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_ADD);
//     ins->Draw_A_Graph(pitch_str.get());

//     static auto rpm_str = std::make_unique<REFEREE_n::CILENT_UI_n::string_data_c>("RPM", rpm, 130, 725, Graph_Color_e::UI_Color_Main, 1, 3, 30);
//     rpm_str->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_ADD);
//     ins->Draw_A_Graph(rpm_str.get());

//     static auto str_hz = std::make_unique<REFEREE_n::CILENT_UI_n::string_data_c>("HZ", Hz, 130, 620, Graph_Color_e::UI_Color_Yellow, 1, 3, 30);
//     str_hz->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_ADD);
//     ins->Draw_A_Graph(str_hz.get());

//     static auto auto_aim_str = std::make_unique<REFEREE_n::CILENT_UI_n::string_data_c>("AUTO", "AUTO:", 797, 260, Graph_Color_e::UI_Color_Main, 1, 3, 30);
//     auto_aim_str->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_ADD);
//     ins->Draw_A_Graph(auto_aim_str.get());


// }

// void dynamic_ui_ope(UI_Draw_c *ins, REFEREE_n::CILENT_UI_n::Graph_Operate_e operate)
// {
// //字符串组
//     // static auto switch_str = std::make_unique<REFEREE_n::CILENT_UI_n::string_data_c>("swit", ON_OFF_STATE[0], 1002, 695, Graph_Color_e::UI_Color_Yellow, 2, 3, 30);
//     // switch_str->configure(operate);
//     // ins->Draw_A_Graph(switch_str.get());

//     // static auto hz_state = std::make_unique<REFEREE_n::CILENT_UI_n::string_data_c>("HZ", fire_hz_state[0], 366, 620, Graph_Color_e::UI_Color_Yellow, 1, 3, 30);
//     // hz_state->configure(operate);
//     // ins->Draw_A_Graph(hz_state.get());

//     // static auto auto_aim_state = std::make_unique<REFEREE_n::CILENT_UI_n::string_data_c>("AUTO", ON_OFF_STATE[0], 1002, 260, Graph_Color_e::UI_Color_Yellow, 1, 3, 30);
//     // auto_aim_state->configure(operate);
//     // ins->Draw_A_Graph(auto_aim_state.get());
// //浮点组
//     static auto pitch_comp = std::make_unique<REFEREE_n::CILENT_UI_n::float_data_c>("PIC", 0, 366, 815, Graph_Color_e::UI_Color_Yellow, 2, 3, 30, 2);
//     pitch_comp->configure(operate);

//     static auto rpm_speed = std::make_unique<REFEREE_n::CILENT_UI_n::float_data_c>("SET", 0, 366, 725, Graph_Color_e::UI_Color_Yellow, 2, 3, 30, 2);
//     rpm_speed->configure(operate);

// //线段组
//     static auto line1 = std::make_unique<REFEREE_n::CILENT_UI_n::LineCommand_c>("143", 600, 100, 800, 500);
//     line1->configure(operate);
// //图形组:待测试
//     static auto fir_state = std::make_unique<REFEREE_n::CILENT_UI_n::RectangleCommand_c>("f", 1002, 695,644,543, Graph_Color_e::UI_Color_Orange, 2, 3);
//     fir_state->configure(operate); //摩擦轮开启状态

//     static auto Auto_state = std::make_unique<REFEREE_n::CILENT_UI_n::RectangleCommand_c>("aa", 1002, 695, 644, 543, Graph_Color_e::UI_Color_Green, 2, 3);
//     Auto_state->configure(operate); //自瞄是否掉线
//     static auto Aim_state = std::make_unique<REFEREE_n::CILENT_UI_n::RectangleCommand_c>("aim", 1002, 695, 644, 543, Graph_Color_e::UI_Color_Cyan, 2, 3);
//     Aim_state->configure(operate); //自瞄是否瞄上
//     static auto Hz_state = std::make_unique<REFEREE_n::CILENT_UI_n::RectangleCommand_c>("Hz", 1002, 695, 644, 543, Graph_Color_e::UI_Color_Yellow, 2, 3);
//     Hz_state->configure(operate); // 弹频状态

//     std::vector<REFEREE_n::CILENT_UI_n::GraphicCommand *> graphic_group;
//     graphic_group.push_back(pitch_comp.get());
//     graphic_group.push_back(rpm_speed.get());
//     graphic_group.push_back(line1.get());
//     graphic_group.push_back(fir_state.get());
//     graphic_group.push_back(Auto_state.get());
//     graphic_group.push_back(Aim_state.get());
//     graphic_group.push_back(Hz_state.get());
//     ins->Draw_Group_Graph(graphic_group);

// }
// void dynamic_ui_update()
// {

// }




}
}