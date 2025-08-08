// #ifndef __BSP_REFEREE_H
// #define __BSP_REFEREE_H

// #include "CRC.h"
// #include "stdint.h"
// // #include "string.h"
// #include "stm32f4xx_hal.h"

// #define Referee_Data_len 128

// // 帧头长度
// #define HEADER_LEN 5
// // 指令长度
// #define CMDID_LEN 2
// // CRC冗余码长度1
// #define CRC16_LEN 2

// // 数据段长度
// #define DATA_STATUS_LEN 11	 //! 比赛状态数据长度(官方有误)
// #define DATA_RESULT_LEN 1	 // 比赛结果数据长度
// #define DATA_ROBOT_HP_LEN 32 //! 比赛机器人血量数据长度(官方有误)
// // #define DATA_DART_STATUS_LEN									3							//飞镖发射状态长度
// #define DATA_EVENT_DATA_LEN 4				// 场地事件数据长度
// #define DATA_SUPPLY_PROJECTILE_ACTION_LEN 4 //! 场地补给站动作标识数据长度(官方有误)
// #define DATA_REFEREE_WARNING_LEN 3			// 裁判警告数据长度
// #define DATA_DART_REMAINING_TIME_LEN 3		// 飞镖发射口倒计时
// #define DATA_ROBOT_STATUS_LEN 13			//! 机器人状态数据(官方有误)
// #define DATA_POWER_HEAT_DATA_LEN 16			//! 实时功率热量数据(官方有误)
// #define DATA_ROBOT_POS_LEN 12				// 机器人位置数据
// #define DATA_BUFF_LEN 1						// 机器人增益数据
// #define DATA_AERIAL_ROBOT_ENERGY_LEN 2		//! 空中机器人能量状态数据,只有空中机器人主控发送(官方有误)
// #define DATA_ROBOT_HURT_LEN 1				// 伤害状态数据
// #define DATA_SHOOT_DATA_LEN 7				//! 实时射击数据(官方有误)
// #define DATA_BULLET_REMAINING_LEN 6			//! 子弹剩余发送数(官方有误)
// #define DATA_RFID_STATUS_LEN 4				// 机器人 RFID 状态
// #define DATA_DART_CLIENT_CMD_LEN 6			// 飞镖机器人客户端指令书
// #define DATA_ROBOT_POSITION_LEN 40
// #define DATA_RADAR_MARK_LEN 6
// #define DATA_SENTRY_INFO_LEN 4
// // #define DATA_STUDENT_INTERACTIVE_HEADER_DATA  6             //UI
// #define DATA_DIY_CONTROLLER 30		 // 自定义控制器
// #define DATA_CLIENT_DOWMLOAD 15		 // 小地图下发位置信息
// #define DATA_PICTURE_TRANSMISSION 12 // 图传遥控信息
// #define DATA_CLIENT_RECEIVE 10		 // 小地图接收位置信息

// // 命令码ID
// #define ID_STATE 0x0001	   // 比赛状态数据
// #define ID_RESULT 0x0002   // 比赛结果数据
// #define ID_ROBOT_HP 0x0003 // 比赛机器人机器人血量数据
// // #define ID_DART_STATUS 												0x0004				//飞镖发射状态
// #define ID_EVENT_DATA 0x0101				// 场地事件数据
// #define ID_SUPPLY_PROJECTILE_ACTION 0x0102	// 场地补给站动作标识数据
// #define ID_SUPPLY_PROJECTILE_BOOKING 0x0103 // 场地补给站预约子弹数据
// #define ID_REFEREE_WARNING 0x0104			// 裁判警告数据
// #define ID_DART_REMAINING_TIME 0x0105		// 飞镖发射口倒计时
// #define ID_ROBOT_STATE 0x0201				// 机器人状态数据
// #define ID_POWER_HEAT_DATA 0x0202			// 实时功率热量数据
// #define ID_ROBOT_POS 0x0203					// 机器人位置数据
// #define ID_BUFF 0x0204						// 机器人增益数据
// #define ID_AERIAL_ROBOT_ENERGY 0x0205		// 空中机器人能量状态数据
// #define ID_ROBOT_HURT 0x0206				// 伤害状态数据
// #define ID_SHOOT_DATA 0x0207				// 实时射击数据
// #define ID_BULLET_REMAINING 0x0208			// 子弹剩余发送数
// #define ID_RFID_STATUS 0x0209				// 机器人 RFID 状态
// #define ID_DART_CLIENT_CMD 0x020A			// 飞镖机器人客户端指令数据
// #define ID_GROUND_ROBOT_POSITION 0X020B
// #define ID_RARD_MRAK_DATA 0X020C
// #define ID_SENTRY 0X020D
// #define ID_RADAR 0x020E

// // #define ID_STUDENT_INTERACTIVE_HEADER_DATA    0x0301       // UI
// #define ID_DIY_CONTROLLER 0x0302	   // 自定义控制器
// #define ID_CLIENT_DOWMLOAD 0x0303	   // 小地图下发位置信息
// #define ID_PICTURE_TRANSMISSION 0x0304 // 图传遥控信息
// #define ID_CLIENT_RECEIVE 0x0305	   // 小地图接收位置信息

// /*比赛状态数据*/ // 0x0001
// typedef __packed struct
// {
// 	uint8_t game_type : 4;
// 	uint8_t game_progress : 4;
// 	uint16_t stage_remain_time;
// 	uint64_t SyncTimeStamp;
// 	uint8_t error;
// } Game_Type_Data;

// /*比赛结果数据*/ // 0x0002
// typedef __packed struct
// {
// 	uint8_t winner;
// 	uint8_t error;
// } Game_Result_t;

// /*血量数据*/ // 0x0003
// typedef __packed struct
// {
// 	uint16_t red_1_robot_HP;
// 	uint16_t red_2_robot_HP;
// 	uint16_t red_3_robot_HP;
// 	uint16_t red_4_robot_HP;
// 	uint16_t red_5_robot_HP;
// 	uint16_t red_7_robot_HP;
// 	uint16_t red_outpost_HP;
// 	uint16_t red_base_HP;
// 	uint16_t blue_1_robot_HP;
// 	uint16_t blue_2_robot_HP;
// 	uint16_t blue_3_robot_HP;
// 	uint16_t blue_4_robot_HP;
// 	uint16_t blue_5_robot_HP;
// 	uint16_t blue_7_robot_HP;
// 	uint16_t blue_outpost_HP;
// 	uint16_t blue_base_HP;
// 	uint8_t error;
// } Robot_Hp_Data;

// /*场地事件数据*/ // 0x0101
// typedef __packed struct
// {
// 	//   uint32_t event_type;
// 	//   uint32_t BloodPoint_1 : 1;
// 	//   uint32_t BloodPoint_2 : 1;
// 	//   uint32_t BloodPoint_3 : 1;
// 	//   uint32_t ENERGY : 3;
// 	//   uint32_t Annular_HighLand : 1;
// 	//   uint32_t R3_or_B3_HighLand : 1;
// 	//   uint32_t R4_or_B4_HighLand : 1;
// 	uint32_t BloodPoint_us_front : 1;
// 	uint32_t BloodPoint_us_inside : 1;
// 	uint32_t BloodPoint_us : 1;
// 	uint32_t Buff_us : 1;
// 	uint32_t BuffSmall_us : 1;
// 	uint32_t BuffBig_us : 1;
// 	uint32_t Upland_Annular_us : 2;
// 	uint32_t Upland_Trapezium_us : 2;
// 	uint32_t Upland_Trapezium_us_ : 2;
// 	uint32_t Virtual_Sheild_us : 7;
// 	uint32_t Dart_hitPost_afterTime : 9;
// 	uint32_t Dart_AimType : 2;
// 	uint32_t BloodPoint_Mid : 2;
// 	uint8_t error;
// } Area_Data;

// /*补给站动作标识*/ // 0x0102
// typedef __packed struct
// {
// 	uint8_t supply_projectile_id;
// 	uint8_t supply_robot_id;
// 	uint8_t supply_projectile_step;
// 	uint8_t supply_projectile_num;
// 	uint8_t error;
// } Supply_Data;

// /*裁判警告信息*/ // 0x0104
// typedef __packed struct
// {
// 	uint8_t level;
// 	uint8_t foul_robot_id;
// 	uint8_t offend_conut;
// 	uint8_t error;
// } Referee_Warning_t;

// /*飞镖发射口数据*/ // 0x0105
// typedef __packed struct
// {
// 	uint8_t dart_remaining_time;
// 	uint16_t latest_aim : 2;
// 	uint16_t aim_count : 3;
// 	uint16_t aim_target : 2;
// 	uint16_t reserved : 9;
// 	uint8_t error;
// } Dart_Launch_Data;

// /*机器人状态数据*/ // 0x0201
// typedef __packed struct
// {
// 	uint8_t robot_id;
// 	uint8_t robot_level;
// 	uint16_t current_HP;
// 	uint16_t maximum_HP;
// 	uint16_t shooter_barrel_cooling_value;
// 	uint16_t shooter_barrel_heat_limit; // 当前枪口热量上限
// 	uint16_t chassis_power_limit;
// 	uint8_t power_management_gimbal_output : 1;
// 	uint8_t power_management_chassis_output : 1;
// 	uint8_t power_management_shooter_output : 1;
// 	uint8_t reserved : 5;
// 	uint8_t error;
// } Robot_Situation_Data;

// /*功率热量数据*/ // 0x0202
// typedef __packed struct
// {
// 	uint16_t chassis_volt;
// 	uint16_t chassis_current;
// 	float chassis_power;
// 	uint16_t chassis_power_buffer;			// 功率缓存
// 	uint16_t shooter_id1_17mm_cooling_heat; // 枪口热量
// 	uint16_t shooter_id2_17mm_cooling_heat;
// 	uint16_t shooter_id1_42mm_cooling_heat;
// 	uint8_t error;
// } Robot_Power_Heat_Data;

// /*机器人位置*/ // 0x0203
// typedef __packed struct
// {
// 	float x;
// 	float y;
// 	float angle;
// 	uint8_t error;
// } Robot_Position_Data;

// /*机器人增益数据*/ // 0x0204
// typedef __packed struct
// {
// 	uint8_t power_rune_buff;
// 	uint8_t error;
// } Area_Buff_Data;

// /*空中机器人能量状态*/ // 0x0205
// typedef __packed struct
// {
// 	uint8_t attack_time;
// 	uint8_t error;
// } UAV_Data;

// /*伤害状态*/ // 0x0206
// typedef __packed struct
// {
// 	uint8_t armor_id : 4;
// 	uint8_t hurt_type : 4;
// 	uint8_t error;
// } Robot_Hurt_Data;

// /*实时射击信息*/ // 0x0207
// typedef __packed struct
// {
// 	uint8_t bullet_type;
// 	uint8_t shooter_id;
// 	uint8_t bullet_freq;
// 	float bullet_speed; // 弹丸初速度
// 	uint8_t error;
// } Robot_Shoot_Data;

// /*子弹剩余发射数*/ // 0x0208
// typedef __packed struct
// {
// 	uint16_t bullet_remaining_num_17mm; // 17mm 子弹剩余发射数目
// 	uint16_t bullet_remaining_num_42mm; // 42mm 子弹剩余发射数目
// 	uint16_t coin_remaining_num;		// 剩余金币数量
// 	uint8_t error;
// } Robot_RaminingBullet_Data;

// /*RFID状态*/ // 0x0209
// typedef __packed struct
// {
// 	uint32_t BaseLand : 1;
// 	uint32_t HighLand_us : 1;
// 	uint32_t HighLand_enm : 1;
// 	uint32_t RorB3_us : 1;
// 	uint32_t RorB3_enm : 1;
// 	uint32_t RorB4_us : 1;
// 	uint32_t RorB4_enm : 1;
// 	uint32_t Buff : 1;
// 	uint32_t OverSlope_before_us : 1;
// 	uint32_t OverSlope_after_us : 1;
// 	uint32_t OverSlope_before_enm : 1;
// 	uint32_t OverSlope_after_enm : 1;
// 	uint32_t Outpost_us : 1;
// 	uint32_t BloodPooint_us : 1;
// 	uint32_t Patrol_area_us : 1;
// 	uint32_t Patrol_area_enm : 1;
// 	uint32_t Resource_area_us : 1;
// 	uint32_t Resource_area_enm : 1;
// 	uint32_t Exchange_area_us : 1;
// 	uint32_t CentralGain_Point : 1;
// 	uint32_t other : 12;
// 	uint8_t error;
// } RFID_Situation_Data;

// /*飞镖机器人客户端指令数据*/ // 0x020A
// typedef __packed struct
// {
// 	uint8_t dart_launch_opening_status;
// 	uint8_t dart_attack_target;
// 	uint16_t target_change_time;
// 	uint16_t operate_launch_cmd_time;
// 	uint8_t error;
// } Dart_Client_Cmd;

// /*机器人位置数据*/
// typedef __packed struct
// {
// 	float hero_x;
// 	float hero_y;
// 	float engineer_x;
// 	float engineer_y;
// 	float standard_3_x;
// 	float standard_3_y;
// 	float standard_4_x;
// 	float standard_4_y;
// 	float standard_5_x;
// 	float standard_5_y;
// 	uint8_t error;
// } ground_robot_position_t;

// /*雷达标记进度*/
// typedef __packed struct
// {
// 	uint8_t mark_hero_progress;
// 	uint8_t mark_engineer_progress;
// 	uint8_t mark_standard_3_progress;
// 	uint8_t mark_standard_4_progress;
// 	uint8_t mark_standard_5_progress;
// 	uint8_t mark_sentry_progress;
// 	uint8_t error;
// } radar_mark_data_t;

// /*交互数据接收信息*/
// typedef __packed struct
// {
// 	uint16_t data_cmd_id;
// 	uint16_t sender_ID;
// 	uint16_t receiver_ID;
// 	// uint16_t data[];
// 	uint8_t error;
// } student_interactive_header_data_t;

// /*学生机器人间通信*/
// // 未使用过，待测试
// // 可用于UI和车间通信
// typedef __packed struct
// {
// 	uint8_t data[1];
// 	uint8_t error;
// } robot_interactive_data_t;

// /*客户端下发信息*/
// typedef __packed struct
// {
// 	float target_position_x;
// 	float target_position_y;
// 	float target_position_z;
// 	uint8_t commd_keyboard;
// 	uint16_t target_robot_ID;
// 	uint8_t error;
// } Robot_Command;

// #define DATA_PICTURE_TRANSMISSION_LEN 12 // 图传遥控信息
// #define ID_PICTURE_TRANSMISSION 0x0304	 // 图传遥控信息
// /*键鼠遥控数据*/
// typedef __packed struct
// {
// 	int16_t mouse_x;
// 	int16_t mouse_y;
// 	int16_t mouse_z; // 鼠标滚轮移动速度
// 	uint8_t left_button_down;
// 	uint8_t right_button_down;
// 	uint8_t Key_W : 1;
// 	uint8_t Key_S : 1;
// 	uint8_t Key_A : 1;
// 	uint8_t Key_D : 1;
// 	uint8_t Key_Shift : 1;
// 	uint8_t Key_Ctil : 1;
// 	uint8_t Key_Q : 1;
// 	uint8_t Key_E : 1;
// 	uint8_t Key_R : 1;
// 	uint8_t Key_F : 1;
// 	uint8_t Key_G : 1;
// 	uint8_t Key_Z : 1;
// 	uint8_t Key_X : 1;
// 	uint8_t Key_C : 1;
// 	uint8_t Key_V : 1;
// 	uint8_t Key_B : 1;
// 	uint16_t other;
// 	uint8_t error;
// } remote_control_t;

// /*客户端接受信息*/
// // 雷达站发送的坐标信息可以被所有己方操作手在第一视角小地图看到。
// typedef __packed struct
// {
// 	uint16_t target_robot_ID;
// 	float target_position_x;
// 	float target_position_y;
// 	uint8_t error;
// } Client_Map_Command_Data;

// typedef __packed struct
// {
// 	uint32_t sentry_bullet : 11;
// 	uint32_t sentry_bullet_count : 4;
// 	uint32_t sentry_blood_count : 4;
// 	uint32_t reserved : 13;
// 	uint8_t error;
// } sentry_info_t;

// /*裁判系统数据*/
// typedef __packed struct
// {
// 	uint8_t RefereeData[256];
// 	uint8_t RealData[45];
// 	int16_t DataLen;
// 	int16_t RealLen;
// 	int16_t Cmd_ID;
// 	uint8_t RECEIVE_FLAG;
// 	Game_Type_Data Game_Status;
// 	Game_Result_t Game_Result;
// 	Robot_Hp_Data Robot_HP;
// 	Area_Data Event_Data;
// 	Supply_Data Supply_Action;
// 	Referee_Warning_t Referee_Warning;
// 	Dart_Launch_Data Dart_Remaining_Time;
// 	Robot_Situation_Data Robot_Status;
// 	Robot_Power_Heat_Data Power_Heat;
// 	Robot_Position_Data Robot_Position;
// 	Area_Buff_Data Buff;
// 	UAV_Data Aerial_Energy;
// 	Robot_Hurt_Data Robot_Hurt;
// 	Robot_Shoot_Data Shoot_Data;
// 	Robot_RaminingBullet_Data Bullet_Num;
// 	RFID_Situation_Data RFID_Status;
// 	Dart_Client_Cmd Dart_Client;
// 	student_interactive_header_data_t Interact_Header;
// 	robot_interactive_data_t Interact_Data;
// 	Robot_Command Client_Data;
// 	remote_control_t Remote_control;
// 	Client_Map_Command_Data ClientMapData;
// 	ground_robot_position_t Robot_Position_Al;
// 	radar_mark_data_t radar_mark;
// 	sentry_info_t sentry;
// } REFEREE_t;

// // 裁判系统初始化
// void referee_uart_init(UART_HandleTypeDef *huart, uint8_t *rx_buffer1, uint8_t *rx_buffer2, uint8_t buff_num);
// void ECF_referee_uart_init(void);
// void REFEREE_UART_IRQHandler(UART_HandleTypeDef *huart);
// // REFEREE_t *(void);

// #endif
