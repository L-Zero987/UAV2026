#ifndef __RM_CILENT_UI_HPP__
#define __RM_CILENT_UI_HPP__

extern "C"
{
#include "stdlib.h"
#include "stdio.h"

#include "freertos.h"
#include "task.h"
#include "queue.h"

#include "stdarg.h"
#include "usart.h"
#include "stm32f4xx_hal.h"
}

#include "bsp_referee.hpp"
#include "user_maths.hpp"
#include <vector>
#include <queue>
#include <memory>

namespace REFEREE_n
{
    namespace CILENT_UI_n
    {

        // #define Robot_ID UI_Data_RobotID_BStandard2
        // #define Cilent_ID UI_Data_CilentID_BStandard2 //机器人角色设置

        typedef signed char int8_t;
        typedef signed short int int16_t;
        typedef signed long long int64_t;

        /* exact-width unsigned integer types */
        typedef unsigned char uint8_t;
        typedef unsigned short int uint16_t;
        typedef unsigned long long uint64_t;
        typedef unsigned char bool_t;
        typedef float fp32;
        typedef double fp64;

// 屏幕分辨率
#define X_MAX 1920
#define Y_MAX 1080
#define UI_HUART huart1 //给裁判系统电管发送数据

#pragma pack(1) // 按1字节对齐

#define __FALSE 100

/****************************开始标志*********************/
#define UI_SOF 0xA5
/****************************CMD_ID数据********************/
#define UI_CMD_Robo_Exchange 0x0301
/****************************内容ID数据********************/
#define UI_Data_ID_Del 0x100
#define UI_Data_ID_Draw1 0x101
#define UI_Data_ID_Draw2 0x102
#define UI_Data_ID_Draw5 0x103
#define UI_Data_ID_Draw7 0x104
#define UI_Data_ID_DrawChar 0x110

#define USER_DEFINE_ID_ASSERT(x) ((x) >= 0x200 && (x) <= 0x2FF) // 自定义数据ID
/****************************红方机器人ID********************/
#define UI_Data_RobotID_RHero 1
#define UI_Data_RobotID_REngineer 2
#define UI_Data_RobotID_RStandard1 3
#define UI_Data_RobotID_RStandard2 4
#define UI_Data_RobotID_RStandard3 5
#define UI_Data_RobotID_RAerial 6
#define UI_Data_RobotID_RSentry 7
#define UI_Data_RobotID_RRadar 9
/****************************蓝方机器人ID********************/
#define UI_Data_RobotID_BHero 101
#define UI_Data_RobotID_BEngineer 102
#define UI_Data_RobotID_BStandard1 103
#define UI_Data_RobotID_BStandard2 104
#define UI_Data_RobotID_BStandard3 105
#define UI_Data_RobotID_BAerial 106
#define UI_Data_RobotID_BSentry 107
#define UI_Data_RobotID_BRadar 109
/**************************红方操作手ID************************/
#define UI_Data_CilentID_RHero 0x0101
#define UI_Data_CilentID_REngineer 0x0102
#define UI_Data_CilentID_RStandard1 0x0103
#define UI_Data_CilentID_RStandard2 0x0104
#define UI_Data_CilentID_RStandard3 0x0105
#define UI_Data_CilentID_RAerial 0x0106
/***************************蓝方操作手ID***********************/
#define UI_Data_CilentID_BHero 0x0165
#define UI_Data_CilentID_BEngineer 0x0166
#define UI_Data_CilentID_BStandard1 0x0167
#define UI_Data_CilentID_BStandard2 0x0168
#define UI_Data_CilentID_BStandard3 0x0169
#define UI_Data_CilentID_BAerial 0x016A
/***************************删除操作***************************/
#define UI_Data_Del_NoOperate 0
#define UI_Data_Del_Layer 1
#define UI_Data_Del_ALL 2
        /***************************图形配置参数__图形操作********************/
        enum class Graph_Operate_e
        {
            UI_Graph_ADD = 1, // 新增
            UI_Graph_Change,  // 修改
            UI_Graph_Del      // 删除
        };
        /***************************图形配置参数__图形类型********************/
        enum class Graphic_Tpye_e
        {
            UI_Graph_Line = 0,      // 直线
            UI_Graph_Rectangle = 1, // 矩形
            UI_Graph_Circle = 2,    // 整圆
            UI_Graph_Ellipse = 3,   // 椭圆
            UI_Graph_Arc = 4,       // 圆弧
            UI_Graph_Float = 5,     // 浮点型
            UI_Graph_Int = 6,       // 整形
            UI_Graph_Char = 7       // 字符型
        };
        /***************************图形配置参数__图形颜色********************/
        enum class Graph_Color_e
        {
            UI_Color_Main = 0, // 红蓝主色
            UI_Color_Yellow,
            UI_Color_Green,
            UI_Color_Orange,
            UI_Color_Purplish_red, // 紫红色
            UI_Color_Pink,
            UI_Color_Cyan, // 青色
            UI_Color_Black,
            UI_Color_White
        };

#define FRAMEHEAD_LEN 7                       // 帧头固定长度
#define FRAMETAIL_LEN 2                       // 帧尾固定长度
#define LAYER_DATA_LEN 2                      // 图层操作长度
#define UI_DATA_OPERATE_LEN 6                 // 数据段头六个字节
#define GRAPH_DATA_LEN 15                     // 绘制图形数据长度
#define STRING_DATA_LEN 45                    // 绘制字符串数据长度
#define ALL_GRAPH_LEN 30                      // 绘制图形数据总发送长度
#define ALL_STRING_LEN 60                     // 绘制字符串数据总发送长度
#define ALL_MANY_GRAPH_LEN(x) (15 + (x) * 15) // 多数据长度

        typedef struct
        {
            uint8_t SOF;          // 起始字节,固定0xA5
            uint16_t Data_Length; // 帧数据长度
            uint8_t Seq;          // 包序号
            uint8_t CRC8;         // CRC8校验值
            uint16_t CMD_ID;      // 命令ID
        } UI_Packhead;            // 帧头

        typedef struct
        {
            uint16_t Data_ID;     // 内容ID
            uint16_t Sender_ID;   // 发送者ID
            uint16_t Receiver_ID; // 接收者ID
        } UI_Data_Operate;        // 操作定义帧

        typedef struct
        {
            uint8_t Delete_Operate; // 删除操作
            uint8_t Layer;          // 删除图层
        } UI_Data_Delete;           // 删除图层帧

        typedef struct
        {
            uint8_t figure_name[3];
            uint32_t operate_tpye : 3;
            uint32_t figure_tpye : 3;
            uint32_t layer : 4;
            uint32_t color : 4;
            uint32_t details_a : 9;
            uint32_t details_b : 9;
            uint32_t width : 10;
            uint32_t start_x : 11;
            uint32_t start_y : 11;
            union
            {
                struct
                {
                    uint32_t details_c : 10;
                    uint32_t details_d : 11;
                    uint32_t details_e : 11;
                } details_cde;
                int details_f;
            } details;

        } interaction_figure_t;
#pragma pack()

        enum class Stauts
        {
            DYNAMIC, // 动态UI(一直刷的)
            STATIC,  // 静态UI(刷一次就好)
            CHANGE   // 变态UI(改变时刷新)
        };

        // ui数据父类
        class GraphicCommand
        {
        public:
            /**
             * @brief  父类构造函数
             * @param  graphic_num_      图像名称
             *         type_             图形类型
             *         color_            颜色
             *         layer_            图层
             *         width_            线宽
             */
            GraphicCommand(char graphic_num_[3], Graphic_Tpye_e type_, Graph_Color_e color_, uint32_t layer_, uint32_t width_) : graph_type(type_),
                                                                                                                                 color(color_),
                                                                                                                                 layer(layer_),
                                                                                                                                 width(width_)
            {
                for (int i = 0; i < 3 && graphic_num_[i] != '\0'; i++)
                    graphic_num[2 - i] = graphic_num_[i];
            }
            char graphic_num[3];                                     // 图形编号
            Graph_Operate_e operate = Graph_Operate_e::UI_Graph_ADD; // 图形操作
            Graphic_Tpye_e graph_type;                               // 图形类型
            Graph_Color_e color;                                     // 图形颜色
            uint32_t layer;                                          // 图层
            uint32_t width;                                          // 线宽
            Stauts status;
            char data[30]; // 实际数据

            interaction_figure_t interaction_figure; // 配置图形数据
            const interaction_figure_t *get_interaction_figure() const { return &interaction_figure; }

            virtual void configure(Graph_Operate_e operate_) = 0; // 绘制图形

            // 配置并设置为新增
            void configure_add()
            {
                configure(Graph_Operate_e::UI_Graph_ADD);
            }

            // 配置并设置为修改
            void configure_change()
            {
                configure(Graph_Operate_e::UI_Graph_Change);
            }

            // 配置并设置为删除
            void configure_del()
            {
                configure(Graph_Operate_e::UI_Graph_Del);
            }

            void set_status_dynamic() { status = Stauts::DYNAMIC; } // 设置为动态
            void set_status_static() { status = Stauts::STATIC; }   // 设置为静态
            void set_color(Graph_Color_e color_) { color = color_; }
            void set_layer(uint32_t layer_) { layer = layer_; }
            void set_width(uint32_t width_) { width = width_; }
        };

        // 直线
        class LineCommand_c : public GraphicCommand
        {
        public:
            /**
             * @brief  直线命令构造函数
             * @param  graphic_num_      图像名称
             *         start_x_,start_y_ 坐标
             *         end_x_,end_y_     坐标
             *         layer_,width_     图层，线宽
             */
            LineCommand_c(char graphic_num_[3], uint32_t start_x_, uint32_t start_y_, uint32_t end_x_, uint32_t end_y_,
                          Graph_Color_e color_ = Graph_Color_e::UI_Color_Green, uint32_t layer_ = 2, uint32_t width_ = 3) : GraphicCommand(graphic_num_, Graphic_Tpye_e::UI_Graph_Line, color_, layer_, width_),
                                                                                                                            coordinate(start_x_, start_y_, end_x_, end_y_)
            {
                set_status_static(); // 默认为静态
            }
            // 坐标
            struct Coordinate_t
            {
                uint32_t start_x;
                uint32_t start_y;
                uint32_t end_x;
                uint32_t end_y;

                Coordinate_t(uint32_t startx, uint32_t starty, uint32_t endx, uint32_t endy) : start_x(startx), start_y(starty), end_x(endx), end_y(endy)
                {
                    user_value_limit(start_x, 0, X_MAX);
                    user_value_limit(start_y, 0, Y_MAX);
                    user_value_limit(end_x, 0, X_MAX);
                    user_value_limit(end_y, 0, Y_MAX);
                }
            };
            Coordinate_t coordinate;

            void configure(Graph_Operate_e operate_) override
            {
                memcpy(&interaction_figure.figure_name, &graphic_num, sizeof(interaction_figure.figure_name));
                interaction_figure.operate_tpye = static_cast<uint32_t>(operate_);
                interaction_figure.figure_tpye = static_cast<uint32_t>(graph_type);
                interaction_figure.layer = layer;
                interaction_figure.color = static_cast<uint32_t>(color);
                interaction_figure.width = width;
                interaction_figure.start_x = coordinate.start_x;
                interaction_figure.start_y = coordinate.start_y;
                interaction_figure.details.details_cde.details_d = coordinate.end_x;
                interaction_figure.details.details_cde.details_e = coordinate.end_y;
            }

            /**
             * @brief  设置坐标
             * @param  x1,y1,x2,y2
             */
            void set_coordinate(uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2)
            {
                coordinate.start_x = x1;
                coordinate.start_y = y1;
                coordinate.end_x = x2;
                coordinate.end_y = y2;
                user_value_limit(coordinate.start_x, 0, X_MAX);
                user_value_limit(coordinate.start_y, 0, Y_MAX);
                user_value_limit(coordinate.end_x, 0, X_MAX);
                user_value_limit(coordinate.end_y, 0, Y_MAX);
            }

        private:
        };

        // 矩形
        class RectangleCommand_c : public GraphicCommand
        {
        public:
            /**
             * @brief  矩形命令构造函数
             * @param  graphic_num_      图像名称
             *         start_x_,start_y_ 坐标
             *         end_x_,end_y_     坐标
             *         layer_,width_     图层，线宽
             */
            RectangleCommand_c(char graphic_num_[3], uint32_t start_x_, uint32_t start_y_, uint32_t end_x_, uint32_t end_y_,
                               Graph_Color_e color_ = Graph_Color_e::UI_Color_Yellow, uint32_t layer_ = 6, uint32_t width_ = 2) : GraphicCommand(graphic_num_, Graphic_Tpye_e::UI_Graph_Rectangle, color_, layer_, width_),
                                                                                                                                  coordinate(start_x_, start_y_, end_x_, end_y_)
            {
                set_status_static(); // 默认为静态
            }
            // 坐标
            struct Coordinate_t
            {
                uint32_t start_x;
                uint32_t start_y;
                uint32_t end_x;
                uint32_t end_y;

                Coordinate_t(uint32_t startx, uint32_t starty, uint32_t endx, uint32_t endy) : start_x(startx), start_y(starty), end_x(endx), end_y(endy)
                {
                    user_value_limit(start_x, 0, X_MAX);
                    user_value_limit(start_y, 0, Y_MAX);
                    user_value_limit(end_x, 0, X_MAX);
                    user_value_limit(end_y, 0, Y_MAX);
                }
            };
            Coordinate_t coordinate;

            /**
             * @brief  绘制图形
             * @param  operate_ 图形操作
             */
            void configure(Graph_Operate_e operate_) override
            {
                memcpy(&interaction_figure.figure_name, &graphic_num, sizeof(interaction_figure.figure_name));
                interaction_figure.operate_tpye = static_cast<uint32_t>(operate_);
                interaction_figure.figure_tpye = static_cast<uint32_t>(graph_type);
                interaction_figure.layer = layer;
                interaction_figure.color = static_cast<uint32_t>(color);
                interaction_figure.width = width;
                interaction_figure.start_x = coordinate.start_x;
                interaction_figure.start_y = coordinate.start_y;
                interaction_figure.details.details_cde.details_d = coordinate.end_x;
                interaction_figure.details.details_cde.details_e = coordinate.end_y;
            }

            /**
             * @brief  设置坐标
             * @param  x1,y1,x2,y2
             */
            void set_coordinate(uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2)
            {
                coordinate.start_x = x1;
                coordinate.start_y = y1;
                coordinate.end_x = x2;
                coordinate.end_y = y2;
                user_value_limit(coordinate.start_x, 0, X_MAX);
                user_value_limit(coordinate.start_y, 0, Y_MAX);
                user_value_limit(coordinate.end_x, 0, X_MAX);
                user_value_limit(coordinate.end_y, 0, Y_MAX);
            }

        private:
        };

        // 正圆
        class CircleCommand : public GraphicCommand
        {
        public:
            /**
             * @brief  正圆命令构造函数
             * @param  graphic_num_      图像名称
             *         start_x_,start_y_ 坐标
             *         radius     半径
             *         layer_,width_     图层，线宽
             */
            CircleCommand(char graphic_num_[3], uint32_t start_x_, uint32_t start_y_, uint32_t radius,
                          Graph_Color_e color_ = Graph_Color_e::UI_Color_Green, uint32_t layer_ = 5, uint32_t width_ = 3) : GraphicCommand(graphic_num_, Graphic_Tpye_e::UI_Graph_Circle, color_, layer_, width_),
                                                                                                                            coordinate(start_x_, start_y_, radius)
            {
                set_status_static(); // 默认为静态
            }
            // 坐标
            struct Coordinate_t
            {
                uint32_t start_x;
                uint32_t start_y;
                uint32_t radius;

                Coordinate_t(uint32_t startx, uint32_t starty, uint32_t radius) : start_x(startx), start_y(starty), radius(radius)
                {
                    user_value_limit(start_x, 0, X_MAX);
                    user_value_limit(start_y, 0, Y_MAX);
                }
            };
            Coordinate_t coordinate;

            /**
             * @brief  绘制图形
             * @param  operate_ 图形操作
             */
            void configure(Graph_Operate_e operate_) override
            {
                memcpy(&interaction_figure.figure_name, &graphic_num, sizeof(interaction_figure.figure_name));
                interaction_figure.operate_tpye = static_cast<uint32_t>(operate_);
                interaction_figure.figure_tpye = static_cast<uint32_t>(graph_type);
                interaction_figure.layer = layer;
                interaction_figure.color = static_cast<uint32_t>(color);
                interaction_figure.width = width;
                interaction_figure.start_x = coordinate.start_x;
                interaction_figure.start_y = coordinate.start_y;
                interaction_figure.details.details_cde.details_c = coordinate.radius;
            }

            void set_coordinate(uint32_t x1, uint32_t y1, uint32_t radius_)
            {
                coordinate.start_x = x1;
                coordinate.start_y = y1;
                coordinate.radius = radius_;
                user_value_limit(coordinate.start_x, 0, X_MAX);
                user_value_limit(coordinate.start_y, 0, Y_MAX);
            }

        private:
        };

        // 椭圆
        class EllipseCommand : public GraphicCommand
        {
        public:
            /**
             * @brief  椭圆命令构造函数
             * @param  graphic_num_      图像名称
             *         start_x_,start_y_ 坐标
             *         len_x_,len_y_     x,y半轴长度
             *         layer_,width_     图层，线宽
             */
            EllipseCommand(char graphic_num_[3], uint32_t start_x_, uint32_t start_y_, uint32_t len_x_, uint32_t len_y_,
                           Graph_Color_e color_ = Graph_Color_e::UI_Color_Black, uint32_t layer_ = 5, uint32_t width_ = 3) : GraphicCommand(graphic_num_, Graphic_Tpye_e::UI_Graph_Ellipse, color_, layer_, width_),
                                                                                                                             coordinate(start_x_, start_y_, len_x_, len_y_)
            {
                set_status_static(); // 默认为静态
            }
            // 坐标
            struct Coordinate_t
            {
                uint32_t start_x;
                uint32_t start_y;
                uint32_t len_x;
                uint32_t len_y;

                Coordinate_t(uint32_t startx, uint32_t starty, uint32_t lenx, uint32_t leny) : start_x(startx), start_y(starty), len_x(lenx), len_y(leny)
                {
                    user_value_limit(start_x, 0, X_MAX);
                    user_value_limit(start_y, 0, Y_MAX);
                    user_value_limit(len_x, 0, X_MAX);
                    user_value_limit(len_y, 0, Y_MAX);
                }
            };
            Coordinate_t coordinate;

            void configure(Graph_Operate_e operate_) override
            {
                memcpy(&interaction_figure.figure_name, &graphic_num, sizeof(interaction_figure.figure_name));
                interaction_figure.operate_tpye = static_cast<uint32_t>(operate_);
                interaction_figure.figure_tpye = static_cast<uint32_t>(graph_type);
                interaction_figure.layer = layer;
                interaction_figure.color = static_cast<uint32_t>(color);
                interaction_figure.width = width;
                interaction_figure.start_x = coordinate.start_x;
                interaction_figure.start_y = coordinate.start_y;
                interaction_figure.details.details_cde.details_d = coordinate.len_x;
                interaction_figure.details.details_cde.details_e = coordinate.len_y;
            }

            void set_coordinate(uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2)
            {
                coordinate.start_x = x1;
                coordinate.start_y = y1;
                coordinate.len_x = x2;
                coordinate.len_y = y2;
                user_value_limit(coordinate.start_x, 0, X_MAX);
                user_value_limit(coordinate.start_y, 0, Y_MAX);
                user_value_limit(coordinate.len_x, 0, X_MAX);
                user_value_limit(coordinate.len_y, 0, Y_MAX);
            }

        private:
        };

        // 圆弧
        class ArcCommand : public GraphicCommand
        {
        public:
            /**
             * @brief  圆弧命令构造函数
             * @param  graphic_num_      图像名称
             *         start_x_,start_y_ 圆心坐标
             *         start_angle,end_angle 圆弧起始角度和结束角度
             *         len_x_,len_y_     x,y半轴长度
             *         layer_,width_     图层，线宽
             */
            ArcCommand(char graphic_num_[3], uint32_t start_x_, uint32_t start_y_, uint32_t start_angle, uint32_t end_angle, uint32_t len_x_, uint32_t len_y_,
                       Graph_Color_e color_ = Graph_Color_e::UI_Color_White, uint32_t layer_ = 5, uint32_t width_ = 3) : GraphicCommand(graphic_num_, Graphic_Tpye_e::UI_Graph_Arc, color_, layer_, width_),
                                                                                                                         coordinate(start_x_, start_y_, start_angle, end_angle, len_x_, len_y_)
            {
                set_status_static(); // 默认为静态
            }

            /**
             * @brief  圆弧命令构造函数(当成圆)
             * @param  graphic_num_      图像名称
             *         start_x_,start_y_ 坐标
             *         radius            半径
             *         len_x_,len_y_     x,y半轴长度
             *         layer_,width_     图层，线宽
             */
            ArcCommand(char graphic_num_[3], uint32_t start_x_, uint32_t start_y_, uint32_t start_angle, uint32_t end_angle, uint32_t radius,
                       Graph_Color_e color_ = Graph_Color_e::UI_Color_White, uint32_t layer_ = 5, uint32_t width_ = 3) : GraphicCommand(graphic_num_, Graphic_Tpye_e::UI_Graph_Arc, color_, layer_, width_),
                                                                                                                         coordinate(start_x_, start_y_, start_angle, end_angle, radius, radius)
            {
                set_status_static(); // 默认为静态
            }

            // 坐标
            struct Coordinate_t
            {
                uint32_t start_x;
                uint32_t start_y;
                uint32_t start_angle;
                uint32_t end_angle;
                uint32_t len_x;
                uint32_t len_y;

                Coordinate_t(uint32_t startx, uint32_t starty, uint32_t startangle, uint32_t endangle, uint32_t lenx, uint32_t leny) : start_x(startx), start_y(starty), start_angle(startangle), end_angle(endangle), len_x(lenx), len_y(leny)
                {
                    user_value_limit(start_x, 0, X_MAX);
                    user_value_limit(start_y, 0, Y_MAX);
                    user_value_limit(start_angle, 0, 360);
                    user_value_limit(end_angle, 0, 360);
                }
            };
            Coordinate_t coordinate;

            void configure(Graph_Operate_e operate_) override
            {
                memset(&interaction_figure, 0, sizeof(interaction_figure));
                memcpy(&interaction_figure.figure_name, &graphic_num, sizeof(interaction_figure.figure_name));
                interaction_figure.operate_tpye = static_cast<uint32_t>(operate_);
                interaction_figure.figure_tpye = static_cast<uint32_t>(graph_type);
                interaction_figure.layer = layer;
                interaction_figure.color = static_cast<uint32_t>(color);
                interaction_figure.width = width;
                interaction_figure.start_x = coordinate.start_x;
                interaction_figure.start_y = coordinate.start_y;
                interaction_figure.details_a = coordinate.start_angle;
                interaction_figure.details_b = coordinate.end_angle;
                interaction_figure.details.details_cde.details_d = coordinate.len_x;
                interaction_figure.details.details_cde.details_e = coordinate.len_y;
            }

            void set_coordinate(uint32_t x1, uint32_t y1, uint32_t a1, uint32_t a2, uint32_t x2, uint32_t y2)
            {
                coordinate.start_x = x1;
                coordinate.start_y = y1;
                coordinate.len_x = x2;
                coordinate.len_y = y2;
                coordinate.start_angle = a1;
                coordinate.end_angle = a2;
                user_value_limit(coordinate.start_x, 0, X_MAX);
                user_value_limit(coordinate.start_y, 0, Y_MAX);
                user_value_limit(coordinate.start_angle, 0, 360);
                user_value_limit(coordinate.end_angle, 0, 360);
            }
            void set_coordinate(uint32_t x1, uint32_t y1, uint32_t a1, uint32_t a2, uint32_t radius)
            {
                coordinate.start_x = x1;
                coordinate.start_y = y1;
                coordinate.len_x = radius;
                coordinate.len_y = radius;
                coordinate.start_angle = a1;
                coordinate.end_angle = a2;
                user_value_limit(coordinate.start_x, 0, X_MAX);
                user_value_limit(coordinate.start_y, 0, Y_MAX);
                user_value_limit(coordinate.start_angle, 0, 360);
                user_value_limit(coordinate.end_angle, 0, 360);
            }

            // 设置圆弧角度
            void set_coordinate(uint32_t a1, uint32_t a2)
            {
                coordinate.start_angle = a1;
                coordinate.end_angle = a2;
                user_value_limit(coordinate.start_angle, 0, 360);
                user_value_limit(coordinate.end_angle, 0, 360);
            }

            // 设置圆弧半径
            void set_radius(uint32_t radius)
            {
                coordinate.len_x = radius;
                coordinate.len_y = radius;
            }

        private:
        };

        // 浮点数据ui   图层 大小 宽度 数据
        class float_data_c : public GraphicCommand
        {
        public:
            float_data_c(char graphic_num_[3], float data_, uint32_t start_x_, uint32_t start_y_,
                         Graph_Color_e color_ = Graph_Color_e::UI_Color_Green, uint32_t layer_ = 6, uint32_t width_ = 4, uint32_t size_ = 20, uint32_t digit_ = 2) : GraphicCommand(graphic_num_, Graphic_Tpye_e::UI_Graph_Float, color_, layer_, width_),
                                                                                                                                                                     data(data_),
                                                                                                                                                                     size(size_),
                                                                                                                                                                     digit(digit_),
                                                                                                                                                                     coordinate(start_x_, start_y_)
            {
                set_status_dynamic();
            }
            float data;     // 实际数据
            uint32_t size;  // 字号
            uint32_t digit; // 小数位数
            // 坐标
            struct Coordinate_t
            {
                uint32_t start_x;
                uint32_t start_y;

                Coordinate_t(uint32_t x, uint32_t y) : start_x(x), start_y(y)
                {
                    user_value_limit(start_x, 0, X_MAX);
                    user_value_limit(start_y, 0, Y_MAX);
                }
            };
            Coordinate_t coordinate;

            void configure(Graph_Operate_e operate_) override
            {
                memcpy(&interaction_figure.figure_name, &graphic_num, sizeof(interaction_figure.figure_name));
                interaction_figure.operate_tpye = static_cast<uint32_t>(operate_);
                interaction_figure.figure_tpye = static_cast<uint32_t>(graph_type);
                interaction_figure.layer = layer;
                interaction_figure.color = static_cast<uint32_t>(color);
                interaction_figure.width = width;
                interaction_figure.start_x = coordinate.start_x;
                interaction_figure.start_y = coordinate.start_y;
                interaction_figure.details_a = size;
                interaction_figure.details_b = digit;
                interaction_figure.details.details_f = data * 1000.0f;
            }

            void set_data(float data_) { data = data_; }
            void set_coordinate(uint32_t x, uint32_t y)
            {
                coordinate.start_x = x;
                coordinate.start_y = y;
            }
            void set_size(uint32_t size_) { size = size_; }
            void set_digit(uint32_t digit_) { digit = digit_; }

        private:
        };

        // 字符串数据
        class string_data_c : public GraphicCommand
        {
        public:
            string_data_c(char graphic_num_[3], const char *data_, uint32_t start_x_, uint32_t start_y_,
                          Graph_Color_e color_ = Graph_Color_e::UI_Color_Green, uint32_t layer_ = 5, uint32_t width_ = 3, uint32_t size_ = 15) : GraphicCommand(graphic_num_, Graphic_Tpye_e::UI_Graph_Char, color_, layer_, width_),
                                                                                                                                                 size(size_),
                                                                                                                                                 len(strlen(data_)),
                                                                                                                                                 coordinate(start_x_, start_y_)
            {
                strcpy(data, data_);
                set_status_dynamic();
            }
            uint32_t size; // 字号
            uint32_t len;  // 字符长度
            // 坐标
            struct Coordinate_t
            {
                uint32_t start_x;
                uint32_t start_y;

                Coordinate_t(uint32_t x, uint32_t y) : start_x(x), start_y(y)
                {
                    user_value_limit(start_x, 0, X_MAX);
                    user_value_limit(start_y, 0, Y_MAX);
                }
            };
            Coordinate_t coordinate;

            void configure(Graph_Operate_e operate_) override
            {
                memcpy(&interaction_figure.figure_name, &graphic_num, sizeof(interaction_figure.figure_name));
                interaction_figure.operate_tpye = static_cast<uint32_t>(operate_);
                interaction_figure.figure_tpye = static_cast<uint32_t>(graph_type);
                interaction_figure.layer = layer;
                interaction_figure.color = static_cast<uint32_t>(color);
                interaction_figure.width = width;
                interaction_figure.start_x = coordinate.start_x;
                interaction_figure.start_y = coordinate.start_y;
                interaction_figure.details_a = size;
                interaction_figure.details_b = len;
            }

            void set_data(const char *data_)
            {
                memset(data, 0, 30);
                strcpy(data, data_);
                len = strlen(data_);
            }
            void set_coordinate(uint32_t x, uint32_t y)
            {
                coordinate.start_x = x;
                coordinate.start_y = y;
            }
            void set_size(uint32_t size_) { size = size_; }
            char *get_data() { return data; }

        private:
        };

        class UI_Draw_c
        {
        private:
            UART_HandleTypeDef *huart_handle;
            unsigned char UI_Seq = 0; // 包序号
            uint16_t sender_id;
            uint16_t receiver_id;
            TickType_t delay_time_ms = 40;

        public:
            UI_Draw_c(UART_HandleTypeDef *huart_handle_) : huart_handle(huart_handle_), UI_Seq(0),delay_time_ms(50) {}

            void Set_robo_id(const uint16_t &sender_id_)
            {
                sender_id = sender_id_;
                if (sender_id == UI_Data_RobotID_BHero)
                    receiver_id = UI_Data_CilentID_BHero;
                else if (sender_id == UI_Data_RobotID_RHero)
                    receiver_id = UI_Data_CilentID_RHero;
                else if (sender_id == UI_Data_RobotID_BEngineer)
                    receiver_id = UI_Data_CilentID_BEngineer;
                else if (sender_id == UI_Data_RobotID_REngineer)
                    receiver_id = UI_Data_CilentID_REngineer;
                else if (sender_id == UI_Data_RobotID_BStandard1) // 根据机器人ID选择发送
                    receiver_id = UI_Data_CilentID_BStandard1;
                else if (sender_id == UI_Data_RobotID_RStandard1)
                    receiver_id = UI_Data_CilentID_RStandard1;
                else if (sender_id == UI_Data_RobotID_BStandard2)
                    receiver_id = UI_Data_CilentID_BStandard2;
                else if (sender_id == UI_Data_RobotID_RStandard2)
                    receiver_id = UI_Data_CilentID_RStandard2;
                else if (sender_id == UI_Data_RobotID_BStandard3)
                    receiver_id = UI_Data_CilentID_BStandard3;
                else if (sender_id == UI_Data_RobotID_RStandard3)
                    receiver_id = UI_Data_CilentID_RStandard3;
                else if (sender_id == UI_Data_RobotID_BAerial)
                    receiver_id = UI_Data_CilentID_BAerial;
                else if (sender_id == UI_Data_RobotID_RAerial)
                    receiver_id = UI_Data_CilentID_RAerial;
            }

            /**
             * @brief 绘制一个图形
             */
            void Draw_A_Graph(const GraphicCommand *graphic_)
            {
                if (graphic_ == nullptr)
                    return;
                if (graphic_->graph_type == Graphic_Tpye_e::UI_Graph_Char)
                {
                    static uint8_t msg[ALL_STRING_LEN];
                    UI_Packhead framehead;
                    framehead.SOF = UI_SOF;
                    framehead.Data_Length = UI_DATA_OPERATE_LEN + STRING_DATA_LEN;
                    framehead.Seq = UI_Seq;
                    framehead.CRC8 = Get_CRC8_Check_Sum((uint8_t *)&framehead, 4, 0xff);
                    framehead.CMD_ID = UI_CMD_Robo_Exchange;

                    UI_Data_Operate data_operate;
                    data_operate.Data_ID = UI_Data_ID_DrawChar;
                    data_operate.Sender_ID = sender_id;
                    data_operate.Receiver_ID = receiver_id;

                    memcpy(msg, &framehead, FRAMEHEAD_LEN);
                    memcpy(msg + FRAMEHEAD_LEN, &data_operate, UI_DATA_OPERATE_LEN);
                    memcpy(msg + FRAMEHEAD_LEN + UI_DATA_OPERATE_LEN, graphic_->get_interaction_figure(), 15);
                    memcpy(msg + FRAMEHEAD_LEN + UI_DATA_OPERATE_LEN + 15, graphic_->data, 30);

                    Append_CRC16_Check_Sum(msg, ALL_STRING_LEN);
                    HAL_UART_Transmit_DMA(huart_handle, msg, ALL_STRING_LEN);
                }
                else
                {
                    static uint8_t msg[ALL_GRAPH_LEN]; // 一个图形数据（15字节）+固定15字节
                    UI_Packhead framehead;
                    framehead.SOF = UI_SOF;
                    framehead.Data_Length = UI_DATA_OPERATE_LEN + GRAPH_DATA_LEN;
                    framehead.Seq = UI_Seq;
                    framehead.CRC8 = Get_CRC8_Check_Sum((uint8_t *)&framehead, 4, 0xff);
                    framehead.CMD_ID = UI_CMD_Robo_Exchange;

                    UI_Data_Operate data_operate;
                    data_operate.Data_ID = UI_Data_ID_Draw1; // 绘制一个
                    data_operate.Sender_ID = sender_id;
                    data_operate.Receiver_ID = receiver_id;

                    memcpy(msg, &framehead, FRAMEHEAD_LEN);
                    memcpy(msg + FRAMEHEAD_LEN, &data_operate, UI_DATA_OPERATE_LEN);
                    memcpy(msg + FRAMEHEAD_LEN + UI_DATA_OPERATE_LEN, graphic_->get_interaction_figure(), GRAPH_DATA_LEN);

                    Append_CRC16_Check_Sum(msg, ALL_GRAPH_LEN);
                    HAL_UART_Transmit_DMA(huart_handle, msg, ALL_GRAPH_LEN);
                }
                UI_Seq++;
                vTaskDelay(delay_time_ms);
            }

            /**
             * @brief 绘制一组图形（不包含字符串）
             * @param graphic_ 图形数组
             * @param num      个数，不能大于7
             */
            void Draw_Group_Graph(const std::vector<GraphicCommand *> &graphic_)
            {
                if (graphic_.size() > 7)
                    return;

                std::vector<GraphicCommand *> graphic_copy = graphic_;

                uint8_t num = 0;
                UI_Data_Operate data_operate;
                switch (graphic_copy.size())
                {
                case 1:
                    data_operate.Data_ID = UI_Data_ID_Draw1; // 绘制一个
                    break;
                case 2:
                    data_operate.Data_ID = UI_Data_ID_Draw2; // 绘制两个
                    break;
                case 4:
                    graphic_copy.push_back(graphic_[0]);
                case 3:
                    graphic_copy.push_back(graphic_[0]); // 占位
                case 5:
                    data_operate.Data_ID = UI_Data_ID_Draw5; // 绘制五个
                    break;
                case 6:
                    graphic_copy.push_back(graphic_[0]);
                case 7:
                    data_operate.Data_ID = UI_Data_ID_Draw7; // 绘制七个
                    break;

                default:
                    break;
                }
                data_operate.Sender_ID = sender_id;
                data_operate.Receiver_ID = receiver_id;
                num = graphic_copy.size();

                uint8_t msg[ALL_MANY_GRAPH_LEN(num)];
                UI_Packhead framehead;
                framehead.SOF = UI_SOF;
                framehead.Data_Length = UI_DATA_OPERATE_LEN + GRAPH_DATA_LEN * num;
                framehead.Seq = UI_Seq;
                framehead.CRC8 = Get_CRC8_Check_Sum((uint8_t *)&framehead, 4, 0xff);
                framehead.CMD_ID = UI_CMD_Robo_Exchange;

                memcpy(msg, &framehead, FRAMEHEAD_LEN);
                memcpy(msg + FRAMEHEAD_LEN, &data_operate, ALL_MANY_GRAPH_LEN(num));
                for (int i = 1; i <= num; i++)
                {
                    memcpy(msg + FRAMEHEAD_LEN + UI_DATA_OPERATE_LEN + GRAPH_DATA_LEN * (i - 1), graphic_copy[i - 1]->get_interaction_figure(), GRAPH_DATA_LEN);
                }

                Append_CRC16_Check_Sum(msg, ALL_MANY_GRAPH_LEN(num));
                HAL_UART_Transmit_DMA(huart_handle, msg, ALL_MANY_GRAPH_LEN(num));
                UI_Seq++;
                vTaskDelay(delay_time_ms);
            }

            // 删除一个图层
            void Del_A_Layer(uint8_t layer_)
            {
                static uint8_t msg[LAYER_DATA_LEN + 15]; // 2 + 固定15字节
                UI_Packhead framehead;
                framehead.SOF = UI_SOF;
                framehead.Data_Length = UI_DATA_OPERATE_LEN + LAYER_DATA_LEN;
                framehead.Seq = UI_Seq;
                framehead.CRC8 = Get_CRC8_Check_Sum((uint8_t *)&framehead, 4, 0xff);
                framehead.CMD_ID = UI_CMD_Robo_Exchange;

                UI_Data_Operate data_operate;
                data_operate.Data_ID = UI_Data_ID_Del;
                data_operate.Sender_ID = sender_id;
                data_operate.Receiver_ID = receiver_id;

                UI_Data_Delete data_delete;
                data_delete.Delete_Operate = UI_Data_Del_Layer;
                data_delete.Layer = layer_;

                memcpy(msg, &framehead, FRAMEHEAD_LEN);
                memcpy(msg + FRAMEHEAD_LEN, &data_operate, UI_DATA_OPERATE_LEN);
                memcpy(msg + FRAMEHEAD_LEN + UI_DATA_OPERATE_LEN, &data_delete, LAYER_DATA_LEN);

                Append_CRC16_Check_Sum(msg, LAYER_DATA_LEN + 15);
                HAL_UART_Transmit_DMA(huart_handle, msg, LAYER_DATA_LEN + 15);

                UI_Seq++;
                vTaskDelay(delay_time_ms);
            }

            // 删除所有图层
            void Del_All_Layer()
            {
                static uint8_t msg[LAYER_DATA_LEN + 15]; // 2 + 固定15字节
                UI_Packhead framehead;
                framehead.SOF = UI_SOF;
                framehead.Data_Length = UI_DATA_OPERATE_LEN + LAYER_DATA_LEN;
                framehead.Seq = UI_Seq;
                framehead.CRC8 = Get_CRC8_Check_Sum((uint8_t *)&framehead, 4, 0xff);
                framehead.CMD_ID = UI_CMD_Robo_Exchange;

                UI_Data_Operate data_operate;
                data_operate.Data_ID = UI_Data_ID_Del;
                data_operate.Sender_ID = sender_id;
                data_operate.Receiver_ID = receiver_id;

                UI_Data_Delete data_delete;
                data_delete.Delete_Operate = UI_Data_Del_ALL;

                memcpy(msg, &framehead, FRAMEHEAD_LEN);
                memcpy(msg + FRAMEHEAD_LEN, &data_operate, UI_DATA_OPERATE_LEN);
                memcpy(msg + FRAMEHEAD_LEN + UI_DATA_OPERATE_LEN, &data_delete, LAYER_DATA_LEN);

                Append_CRC16_Check_Sum(msg, LAYER_DATA_LEN + 15);
                HAL_UART_Transmit_DMA(huart_handle, msg, LAYER_DATA_LEN + 15);

                UI_Seq++;
                vTaskDelay(delay_time_ms);
            }
        };

    }; // CILENT_UI_n
}; // REFEREE_n
#endif
