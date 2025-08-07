#ifndef UI_TASK_HPP
#define UI_TASK_HPP

#include "rm_cilent_ui_.hpp"

namespace REFEREE_n
{
    namespace CILENT_UI_n
    {
        void Cilent_UI_Init();
        void Cilent_UI_Task();

#define CROSSHAIR_SIGHT_X (X_MAX / 2) // 准星X坐标
#define CROSSHAIR_SIGHT_Y (Y_MAX / 2) // 准星Y坐标
#define DIS_AIM_RADIO 40              // 未瞄准时的半径
#define AIM_RADIO 20                  // 成功瞄准的半径

        /*
        //单发模式准星
           |
        -- * --
           |
        */

        /*
        //连发模式准星

        (  *  )

        */
        class Crosshair
        {
        public:
            Crosshair()
            {
                radio = DIS_AIM_RADIO;
                cross = std::make_unique<REFEREE_n::CILENT_UI_n::CircleCommand>((char*)"001", CROSSHAIR_SIGHT_X, CROSSHAIR_SIGHT_Y, 1, crosshair_color, 3, 7);
                arc1 = std::make_unique<REFEREE_n::CILENT_UI_n::ArcCommand>((char*)"002", CROSSHAIR_SIGHT_X, CROSSHAIR_SIGHT_Y, auto_arc_start_angle[0], auto_arc_start_angle[0] + arc_len, radio, crosshair_color, 3, 7);
                arc2 = std::make_unique<REFEREE_n::CILENT_UI_n::ArcCommand>((char*)"003", CROSSHAIR_SIGHT_X, CROSSHAIR_SIGHT_Y, auto_arc_start_angle[1], auto_arc_start_angle[1] + arc_len, radio, crosshair_color, 3, 7);
                arc3 = std::make_unique<REFEREE_n::CILENT_UI_n::ArcCommand>((char*)"004", CROSSHAIR_SIGHT_X, CROSSHAIR_SIGHT_Y, auto_arc_start_angle[2], auto_arc_start_angle[2] + arc_len, radio, crosshair_color, 3, 7);
                arc4 = std::make_unique<REFEREE_n::CILENT_UI_n::ArcCommand>((char *)"005", CROSSHAIR_SIGHT_X, CROSSHAIR_SIGHT_Y, auto_arc_start_angle[3], auto_arc_start_angle[3] + arc_len, radio, crosshair_color, 3, 7);

                graphic_group.push_back(cross.get());
                graphic_group.push_back(arc1.get());
                graphic_group.push_back(arc2.get());
                graphic_group.push_back(arc3.get());
                graphic_group.push_back(arc4.get());
            }

            // 设置单发连发模式
            void Set_Fire_Mode(uint8_t is_semi_)
            {
                static uint8_t last_is_semi = 0;
                is_semi = is_semi_;
                if (is_semi != last_is_semi)
                {
                    ui_static = is_semi == 1 ? 2 : 0;
                    over_radio = 0;
                }
                last_is_semi = is_semi;
            }

            // 设置摩擦轮开启状态
            void Set_fric_status(uint8_t status_)
            {
                fric_status = status_;
            }

            // 设置自瞄状态
            void Set_Aim_Status(uint8_t status_)
            {
                aim_status = status_;
                switch (aim_status)
                {
                case 0:
                    radio = DIS_AIM_RADIO;
                    over_radio = DIS_AIM_RADIO;
                    crosshair_color = REFEREE_n::CILENT_UI_n::Graph_Color_e::UI_Color_White;
                    break;
                case 1:
                    radio = DIS_AIM_RADIO;
                    over_radio = DIS_AIM_RADIO;
                    crosshair_color = REFEREE_n::CILENT_UI_n::Graph_Color_e::UI_Color_Yellow;
                    break;
                case 2:
                    radio = AIM_RADIO;
                    over_radio = AIM_RADIO;
                    crosshair_color = REFEREE_n::CILENT_UI_n::Graph_Color_e::UI_Color_Purplish_red;
                    break;
                default:
                    break;
                }
            }

            // 初始化添加UI
            std::vector<REFEREE_n::CILENT_UI_n::GraphicCommand *> crosshair_init()
            {
                crosshair_update();

                if (fric_status == 1)
                    for (auto i : graphic_group)
                        i->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_ADD);
                else
                    cross->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_ADD);
                return graphic_group;
            }

            // 更新UI TOTEXT
            std::vector<REFEREE_n::CILENT_UI_n::GraphicCommand *> crosshair_update()
            {
                // 设置颜色
                for (auto i : graphic_group)
                {
                    i->set_color(crosshair_color);
                }

                static uint8_t last_fric_status = 0;
                // 设置准星样式
                switch (ui_static)
                {
                case 0:
                    over_radio += radio_step;
                    user_value_limit(over_radio, 0, radio);
                    if (over_radio == radio)
                        ui_static = 1;
                case 1:
                    arc1->set_coordinate(auto_arc_start_angle[0], auto_arc_start_angle[0] + arc_len);
                    arc2->set_coordinate(auto_arc_start_angle[1], auto_arc_start_angle[1] + arc_len);
                    arc3->set_coordinate(auto_arc_start_angle[2], auto_arc_start_angle[2] + arc_len);
                    arc4->set_coordinate(auto_arc_start_angle[3], auto_arc_start_angle[3] + arc_len);

                    arc1->set_width(auto_arc_width);
                    arc2->set_width(auto_arc_width);
                    arc3->set_width(auto_arc_width);
                    arc4->set_width(auto_arc_width);

                    arc1->set_radius(over_radio);
                    arc2->set_radius(over_radio);
                    arc3->set_radius(over_radio);
                    arc4->set_radius(over_radio);
                    break;
                case 2:
                    over_radio += radio_step;
                    user_value_limit(over_radio, 0, radio);
                    if (over_radio == radio)
                        ui_static = 3;
                case 3:
                    arc1->set_coordinate(semi_arc_angle[0] - 3, 3);
                    arc2->set_coordinate(semi_arc_angle[1] - 3, semi_arc_angle[1] + 3);
                    arc3->set_coordinate(semi_arc_angle[2] - 3, semi_arc_angle[2] + 3);
                    arc4->set_coordinate(semi_arc_angle[3] - 3, semi_arc_angle[3] + 3);

                    arc1->set_width(semi_arc_width);
                    arc2->set_width(semi_arc_width);
                    arc3->set_width(semi_arc_width);
                    arc4->set_width(semi_arc_width);

                    arc1->set_radius(over_radio);
                    arc2->set_radius(over_radio);
                    arc3->set_radius(over_radio);
                    arc4->set_radius(over_radio);
                default:
                    break;
                }

                // // 摩擦轮开启才显示
                if (last_fric_status == 0 && fric_status == 1)
                {
                    for (auto i : graphic_group)
                        i->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_ADD);
                    ui_static = is_semi == 1 ? 2 : 0;
                    over_radio = 0;
                }
                else if (fric_status == 1)
                {
                    for (auto i : graphic_group)
                        i->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_Change);
                }
                else
                {
                    for (auto i : graphic_group)
                        i->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_Del);
                    cross->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_Change);
                }
                last_fric_status = fric_status;
                return graphic_group;
            }

        private:
            std::unique_ptr<REFEREE_n::CILENT_UI_n::CircleCommand> cross; // 准星
            std::unique_ptr<REFEREE_n::CILENT_UI_n::ArcCommand> arc1;     // 辅助圆弧
            std::unique_ptr<REFEREE_n::CILENT_UI_n::ArcCommand> arc2;
            std::unique_ptr<REFEREE_n::CILENT_UI_n::ArcCommand> arc3;
            std::unique_ptr<REFEREE_n::CILENT_UI_n::ArcCommand> arc4;

            std::vector<REFEREE_n::CILENT_UI_n::GraphicCommand *> graphic_group;

            uint32_t radio = DIS_AIM_RADIO; // 准星半径
            REFEREE_n::CILENT_UI_n::Graph_Color_e crosshair_color = REFEREE_n::CILENT_UI_n::Graph_Color_e::UI_Color_White;
            // 连发准星配置
            const uint32_t arc_len = 70;       // 每条圆弧的弧长
            const uint32_t auto_arc_width = 7; // 每条圆弧的弧长
            uint32_t auto_arc_start_angle[4] = {10, 100, 190, 280};
            // 单发准星配置
            const uint32_t semi_arc_width = 20; // 每条圆弧线宽
            uint32_t semi_arc_angle[4] = {360, 90, 180, 270};
            // 调用配置
            uint8_t is_semi = 0;     // 单发连发指示
            uint8_t ui_static = 1;   // 准星ui刷新状态 0：连发过度，1：连发，2：单发过度，3：单发
            uint8_t aim_status = 0;  // 自瞄状态 0：未开启自瞄，1：开启自瞄但是没瞄上，2：开启自瞄且瞄上了
            uint8_t fric_status = 0; // 摩擦轮开启状态
            // 准星膨胀
            uint32_t radio_step = 10; // 半径扩展步长
            uint32_t over_radio;
        };

        class state_ui_c
        {
        public:
            state_ui_c()
            {

                // 摩擦轮
                fir_state_str = std::make_unique<REFEREE_n::CILENT_UI_n::string_data_c>((char*)"Fir", fric, 705, 830, Graph_Color_e::UI_Color_White, 2, 3, 30);
                fir_state_r = std::make_unique<REFEREE_n::CILENT_UI_n::RectangleCommand_c>((char *)"ir", 690, 790, 820, 840, Graph_Color_e::UI_Color_Yellow, 4, 8);
                // 自瞄是否开启
                auto_state_str = std::make_unique<REFEREE_n::CILENT_UI_n::string_data_c>((char*)"AUT", AUTO, 83, 653, Graph_Color_e::UI_Color_Green, 2, 3, 30);
                auto_state_r = std::make_unique<REFEREE_n::CILENT_UI_n::RectangleCommand_c>((char *)"AU", 60, 615, 220, 660, Graph_Color_e::UI_Color_Yellow, 4, 8);
                // 是否瞄上
                aim_state_str = std::make_unique<REFEREE_n::CILENT_UI_n::string_data_c>((char *)"Aim", aim, 260, 653, Graph_Color_e::UI_Color_Green, 2, 3, 30);
                aim_state_r = std::make_unique<REFEREE_n::CILENT_UI_n::RectangleCommand_c>((char *)"aim", 350, 660, 250, 615, Graph_Color_e::UI_Color_Yellow, 4, 8);
                // 弹频状态
                // Hz_str = std::make_unique<REFEREE_n::CILENT_UI_n::string_data_c>("HZ", Hz, 80, 580, Graph_Color_e::UI_Color_Green, 2, 3, 30);
                Hz_state_str = std::make_unique<REFEREE_n::CILENT_UI_n::string_data_c>((char *)"HZs", fire_hz_state[0], 80, 580, Graph_Color_e::UI_Color_Green, 2, 3, 30);
                // Hz_state_r = std::make_unique<REFEREE_n::CILENT_UI_n::RectangleCommand_c>("Hz", 76, 518, 306, 582, Graph_Color_e::UI_Color_Yellow, 4, 8);
                // 以下为加的

                picth_str = std::make_unique<REFEREE_n::CILENT_UI_n::string_data_c>((char*)"PIC", pit, 83, 815, Graph_Color_e::UI_Color_Yellow, 7, 3, 30);
                pitch_comp = std::make_unique<REFEREE_n::CILENT_UI_n::float_data_c>((char*)"PI", 0, 366, 815, Graph_Color_e::UI_Color_Yellow, 6, 3, 30, 2);

                rpm_str = std::make_unique<REFEREE_n::CILENT_UI_n::string_data_c>((char *)"RPM", rpm, 83, 725, Graph_Color_e::UI_Color_Yellow, 7, 3, 30);
                rpm_speed = std::make_unique<REFEREE_n::CILENT_UI_n::float_data_c>((char *)"RP", 0, 366, 725, Graph_Color_e::UI_Color_Yellow, 6, 3, 30, 2);

                graphic_group.push_back(pitch_comp.get());
                graphic_group.push_back(rpm_speed.get());
                graphic_group.push_back(fir_state_r.get());
                graphic_group.push_back(auto_state_r.get());
                graphic_group.push_back(aim_state_r.get());
                // graphic_group.push_back(Hz_state_r.get());
            }

            void setcor(float x1, float y1, float x2, float y2)
            {
                // fir_state_r->set_coordinate(x1, y1, x2, y2);
                // auto_state_r->set_coordinate(x1, y1, x2, y2);
                aim_state_r->set_coordinate(x1, y1, x2, y2);
            }
            void set_fir_state(uint8_t state)
            {
                // 0关1开
                fir_state_r->set_color(state == 0 ? REFEREE_n::CILENT_UI_n::Graph_Color_e::UI_Color_Yellow : REFEREE_n::CILENT_UI_n::Graph_Color_e::UI_Color_Main);
            }
            void set_auto_state(uint8_t state)
            {
                auto_state_r->set_color(state == 0 ? REFEREE_n::CILENT_UI_n::Graph_Color_e::UI_Color_Yellow : REFEREE_n::CILENT_UI_n::Graph_Color_e::UI_Color_Main);
            }
            void set_aim_state(uint8_t state) // 0关1开
            {
                aim_state_r->set_color(state == 0 ? REFEREE_n::CILENT_UI_n::Graph_Color_e::UI_Color_Yellow : REFEREE_n::CILENT_UI_n::Graph_Color_e::UI_Color_Main);
            }
            void set_hz_state(uint8_t state)
            {
                // Hz_state_r->set_color(state == 0 ? REFEREE_n::CILENT_UI_n::Graph_Color_e::UI_Color_Main : REFEREE_n::CILENT_UI_n::Graph_Color_e::UI_Color_Yellow);
                Hz_state_str->set_data(fire_hz_state[state]);
                Hz_state_str->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_Change);
            }

            REFEREE_n::CILENT_UI_n::GraphicCommand *aim_state_update()
            {
                aim_state_r->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_Change);
                return aim_state_r.get();
            }
            REFEREE_n::CILENT_UI_n::GraphicCommand *fir_state_update()
            {
                fir_state_r->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_Change);
                return fir_state_r.get();
            }
            REFEREE_n::CILENT_UI_n::GraphicCommand *auto_state_update()
            {
                auto_state_r->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_Change);
                return auto_state_r.get();
            }
            REFEREE_n::CILENT_UI_n::string_data_c *Hz_state_update()
            {
                Hz_state_str->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_Change);
                return Hz_state_str.get();
            }
            REFEREE_n::CILENT_UI_n::string_data_c *fir_string_init()
            {
                fir_state_str->set_data("FRIC");
                fir_state_str->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_ADD);
                return fir_state_str.get();
            }
            REFEREE_n::CILENT_UI_n::string_data_c *auto_string_init()
            {
                auto_state_str->set_data("AUTO");
                auto_state_str->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_ADD);
                return auto_state_str.get();
            }
            // REFEREE_n::CILENT_UI_n::string_data_c *Hz_string_init()
            // {
            //     Hz_str->set_data("FIRE_HZ:");
            //     Hz_str->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_ADD);
            //     return Hz_str.get();
            // }
            REFEREE_n::CILENT_UI_n::string_data_c *Hz_state_init()
            {
                // Hz_state_str->set_data("LOW_HZ");
                Hz_state_str->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_ADD);
                return Hz_state_str.get();
            }
            REFEREE_n::CILENT_UI_n::string_data_c *aim_string_init()
            {
                aim_state_str->set_data("AIM");
                aim_state_str->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_ADD);
                return aim_state_str.get();
            }
            std::vector<REFEREE_n::CILENT_UI_n::GraphicCommand *> all_state_update()
            {
                fir_state_r->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_Change);
                auto_state_r->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_Change);
                aim_state_r->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_Change);
                rpm_speed->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_Change);
                rpm_speed.get()->set_data(rpm_speed->data);
                pitch_comp->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_Change);

                // Hz_state_r->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_Change);
                return graphic_group;
            }
            std::vector<REFEREE_n::CILENT_UI_n::GraphicCommand *> all_state_init()
            {
                fir_state_r->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_ADD);
                auto_state_r->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_ADD);
                aim_state_r->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_ADD);
                pitch_comp->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_ADD);
                rpm_speed->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_ADD);
                // Hz_state_r->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_ADD);
                return graphic_group;
            }
            void set_pitch_comp(float data)
            {
                pitch_comp->set_data(data);
                // pitch_comp->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_Change);
            }
            void set_rpm_speed(float data)
            {
                rpm_speed->set_data(data);
                // rpm_speed->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_Change);
            }
            REFEREE_n::CILENT_UI_n::float_data_c *pitch_comp_update()
            {
                pitch_comp->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_Change);
                return pitch_comp.get();
            }
            REFEREE_n::CILENT_UI_n::float_data_c *rpm_speed_update()
            {
                rpm_speed->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_Change);
                return rpm_speed.get();
            }
            REFEREE_n::CILENT_UI_n::float_data_c *pitch_comp_init()
            {
                // pitch_comp->set_data(0);
                pitch_comp->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_ADD);
                return pitch_comp.get();
            }
            REFEREE_n::CILENT_UI_n::float_data_c *rpm_speed_init()
            {
                rpm_speed->set_data(6100);
                rpm_speed->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_ADD);
                return rpm_speed.get();
            }

            REFEREE_n::CILENT_UI_n::string_data_c *pitch_string_init()
            {
                picth_str->set_data("PITCH:");
                picth_str->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_ADD);
                return picth_str.get();
            }
            REFEREE_n::CILENT_UI_n::string_data_c *rpm_string_init()
            {
                rpm_str->set_data("RPM:  ");
                rpm_str->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_ADD);
                return rpm_str.get();
            }

        private:
            std::unique_ptr<REFEREE_n::CILENT_UI_n::string_data_c> picth_str;
            std::unique_ptr<REFEREE_n::CILENT_UI_n::string_data_c> rpm_str;
            std::unique_ptr<REFEREE_n::CILENT_UI_n::float_data_c> pitch_comp;
            std::unique_ptr<REFEREE_n::CILENT_UI_n::float_data_c> rpm_speed;

            const char *pit = "PITCH:";
            const char *rpm = "RPM:  ";
            // 以上为加的
            std::unique_ptr<REFEREE_n::CILENT_UI_n::string_data_c> fir_state_str;
            std::unique_ptr<REFEREE_n::CILENT_UI_n::string_data_c> auto_state_str;
            std::unique_ptr<REFEREE_n::CILENT_UI_n::string_data_c> aim_state_str;
            std::unique_ptr<REFEREE_n::CILENT_UI_n::string_data_c> Hz_state_str;
            // std::unique_ptr<REFEREE_n::CILENT_UI_n::string_data_c> Hz_str;

            std::unique_ptr<REFEREE_n::CILENT_UI_n::RectangleCommand_c> fir_state_r;
            std::unique_ptr<REFEREE_n::CILENT_UI_n::RectangleCommand_c> auto_state_r;
            std::unique_ptr<REFEREE_n::CILENT_UI_n::RectangleCommand_c> aim_state_r;

            std::vector<REFEREE_n::CILENT_UI_n::GraphicCommand *> graphic_group;

            const char *AUTO = "AUTO";
            const char *aim = "AIM";
            const char *fric = "FRIC";
            const char *Hz = "FIRE_HZ:";
            const char *fire_hz_state[2] = {
                "LOW_HZ ", //
                "HIGH_HZ", //
            };
        };

        class aware_c
        {
        public:
            aware_c()
            {
                aware_state_r = std::make_unique<REFEREE_n::CILENT_UI_n::RectangleCommand_c>((char *)"a", 1500, 553, 1700, 750, Graph_Color_e::UI_Color_Orange, 4, 8);

                graphic_group.push_back(aware_state_r.get());
            }
            void change_aware(uint16_t game_time) // 剩余时间
            {
                for (uint8_t i = 0; i < 5; i++)
                {
                    if ((game_time > need_show_time[i] - 20) && (game_time < need_show_time[i]))
                    {
                        static bool change = false;

                        need_show = 1; // 闪烁展示
                        aware_state_r->set_color(change == true ? REFEREE_n::CILENT_UI_n::Graph_Color_e::UI_Color_Orange : REFEREE_n::CILENT_UI_n::Graph_Color_e::UI_Color_Cyan);
                        change = !change;
                        break;
                    }
                    else if ((need_show_time[i] > game_time) && (game_time < need_show_time[i] + 30))
                    {                  // 常亮展示大小符
                        need_show = 1; // 闪烁展示

                        aware_state_r->set_color(REFEREE_n::CILENT_UI_n::Graph_Color_e::UI_Color_Main);
                    }
                }
            }

            std::vector<REFEREE_n::CILENT_UI_n::GraphicCommand *> _state_init()
            {
                if (need_show)
                {
                    aware_state_r->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_ADD);
                }
                else
                {
                    aware_state_r->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_Del);
                }
                return graphic_group;
            }
            std::vector<REFEREE_n::CILENT_UI_n::GraphicCommand *> _state_update()
            {
                
                if (need_show)
                {
                    aware_state_r->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_Change);
                }
                else
                {
                    aware_state_r->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_Del);
                }
                return graphic_group;
            }
             REFEREE_n::CILENT_UI_n::GraphicCommand *aware_delete()
            {
                aware_state_r->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_Del);
                return aware_state_r.get();
            }

        private:
            uint8_t need_show = 0;
            const uint16_t need_show_time[5] = {
                360, 270, 180, 105, 30};

            // 6min 4.5min
            // 3min 1min45s 30s
            std::unique_ptr<REFEREE_n::CILENT_UI_n::RectangleCommand_c> aware_state_r;
            std::vector<REFEREE_n::CILENT_UI_n::GraphicCommand *> graphic_group;
        };

        class dynamic_data_c
        {
        public:
            dynamic_data_c()
            {
                picth_str = std::make_unique<REFEREE_n::CILENT_UI_n::string_data_c>((char*)"PIC", pit, 83, 815, Graph_Color_e::UI_Color_Yellow, 7, 3, 30);
                pitch_comp = std::make_unique<REFEREE_n::CILENT_UI_n::float_data_c>((char *)"PI", 0, 366, 815, Graph_Color_e::UI_Color_Yellow, 6, 3, 30, 2);
                rpm_str = std::make_unique<REFEREE_n::CILENT_UI_n::string_data_c>((char*)"RPM", rpm, 83, 725, Graph_Color_e::UI_Color_Yellow, 7, 3, 30);
                rpm_speed = std::make_unique<REFEREE_n::CILENT_UI_n::float_data_c>((char *)"RP", 0, 366, 725, Graph_Color_e::UI_Color_Yellow, 6, 3, 30, 2);
                graphic_group.push_back(pitch_comp.get());
                graphic_group.push_back(rpm_speed.get());
            }
            void set_pitch_comp(float data)
            {
                pitch_comp->set_data(data);
                // pitch_comp->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_Change);
            }
            void set_rpm_speed(float data)
            {
                rpm_speed->set_data(data);
                // rpm_speed->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_Change);
            }
            REFEREE_n::CILENT_UI_n::float_data_c *pitch_comp_update()
            {
                pitch_comp->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_Change);
                return pitch_comp.get();
            }
            REFEREE_n::CILENT_UI_n::float_data_c *rpm_speed_update()
            {
                rpm_speed->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_Change);
                return rpm_speed.get();
            }
            std::vector<REFEREE_n::CILENT_UI_n::GraphicCommand *> all_data_update()
            {
                rpm_speed->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_Change);
                pitch_comp->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_Change);

                return graphic_group;
            }
            std::vector<REFEREE_n::CILENT_UI_n::GraphicCommand *> all_data_init()
            {
                // pitch_comp->set_data(0);
                // rpm_speed->set_data(0);
                pitch_comp->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_ADD);
                rpm_speed->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_ADD);
                return graphic_group;
            }
            REFEREE_n::CILENT_UI_n::float_data_c *pitch_comp_init()
            {
                // pitch_comp->set_data(0);
                pitch_comp->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_ADD);
                return pitch_comp.get();
            }
            REFEREE_n::CILENT_UI_n::float_data_c *rpm_speed_init()
            {
                rpm_speed->set_data(6100);
                rpm_speed->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_ADD);
                return rpm_speed.get();
            }

            REFEREE_n::CILENT_UI_n::string_data_c *pitch_string_init()
            {
                picth_str->set_data("PITCH:");
                picth_str->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_ADD);
                return picth_str.get();
            }
            REFEREE_n::CILENT_UI_n::string_data_c *rpm_string_init()
            {
                rpm_str->set_data("RPM:  ");
                rpm_str->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_ADD);
                return rpm_str.get();
            }

        private:
            std::unique_ptr<REFEREE_n::CILENT_UI_n::string_data_c> picth_str;
            std::unique_ptr<REFEREE_n::CILENT_UI_n::string_data_c> rpm_str;
            std::unique_ptr<REFEREE_n::CILENT_UI_n::float_data_c> pitch_comp;
            std::unique_ptr<REFEREE_n::CILENT_UI_n::float_data_c> rpm_speed;
            std::vector<REFEREE_n::CILENT_UI_n::GraphicCommand *> graphic_group;

            const char *pit = "PITCH:";
            const char *rpm = "RPM:  ";
        };
    }
}
#endif // !UI_TASK
//烧饼决策，
//能量机关
//