/*******************************************************************
版权声明：       HITCRT-iHiter(哈工大竞技机器队RoboMaster iHiter战队)
文件名：         rm_rs_types.h
最近修改日期：   2021.5.26
版本：           v1.1
--------------------------------------------------------------------
模块描述：       该模块定义裁判系统通讯接收的相关结构体。
--------------------------------------------------------------------
修改记录：
作者         		时间            版本		说明
Zhou Zhicheng		2021.5.26		1.1			基于20210430版官方通讯协议进行移植
												飞镖发射标志位被移除
********************************************************************/

#ifndef RM_RS_V2021_TYPES_H
#define RM_RS_V2021_TYPES_H

#include "stdint.h"


#define	Red_1_Hero		1
#define	Red_2_Engineer	2
#define Red_3_Standard 	3
#define Red_4_Standard  4
#define Red_5_Standard  5
#define Red_6_Aerial    6
#define Red_7_Sentry    7
#define	Red_9_Radar     9
    
#define	Blue_1_Hero     101
#define Blue_2_Engineer 102
#define Blue_3_Standard 103
#define Blue_4_Standard 104
#define Blue_5_Standard 105
#define Blue_6_Aerial   106
#define Blue_7_Sentry   107
#define	Blue_9_Radar    109


#define GameStatusID    			0x0001
#define GameResultID    			0x0002
#define GameRobotHPID   			0x0003
#define DartStatusID    			0x0004
#define ICRABuffDebuffZoneStatusID  0x0005
#define EventDataID     			0x0101
#define SupplyProjectileActionID    0x0102
#define ApplySupplyProjectoleID		0x0103
#define RefereeWaringID     		0x0104
#define DartRemainingTimeID     	0x0105
#define GameRobotStatusID       	0x0201
#define PowerHeatDataID     		0x0202
#define GameRobotPosID      		0x0203
#define BuffID      				0x0204
#define AerialRobotEnergyID     	0x0205
#define RobotHurtID     			0x0206
#define ShootDataID     			0x0207
#define BulletRemainingID   		0x0208
#define RFIDStatusID        		0x0209
#define DartClientCmdID     		0x020A
#define RobotPosID                  0x020B
#define SentryDecisionID            0x020D
#define RobotInteractiveDataID  	0x0301
#define RobotCommandID            0x0303

#define ClientCustomGraphicDeleteID		0x0100
#define ClientCustomGraphicSingleID		0x0101
#define ClientCustomGraphicDoubleID		0x0102
#define ClientCustomGraphicFiveID		0x0103
#define ClientCustomGraphicSevenID		0x0104
#define ClientCustomCharacterID			0x0110

#define Custom_Data_Length_MAX			113

#ifdef __cplusplus
#pragma pack(1)
#endif 

/* 裁判系统数据接收体 */
/** 比赛状态数据：0x0001    发送频率：1Hz
  * @param	game_type	比赛类型：	1-RoboMaster机甲大师赛
  *									2-RoboMaster机甲大师单项赛
  *									3-ICRA RoboMaster人工智能挑战赛
  *									4-RoboMaster联盟赛3V3
  *									5-RoboMaster联盟赛1V1
  *			game_progress	当前比赛阶段：	0-未开始比赛
  *											1-准备阶段
  *											2-自检阶段
  *											3-5s倒计时
  *											4-对战中
  *											5-比赛结算中
  *			stage_remain_time	当前阶段剩余时间，单位s
  *			SyncTimeStamp		机器人接收到该指令的精确Unix时间，当机载端收到有效的NTP服务器授时后生效
  */
typedef  struct {
	uint8_t game_type: 4;
	uint8_t game_progress: 4;
	uint16_t stage_remain_time;
	uint64_t SyncTimeStamp;
} ext_game_status_t;



/** 比赛结果数据：0x0002    发送频率：比赛结束后发送
  *	@param	winner	0-平局	1-红方胜利	2-蓝方胜利
  */
typedef  struct {
	uint8_t winner;
} ext_game_result_t;



/** 机器人血量数据：0x0003   发送频率：1Hz
* @brief	提供各队机器人血量数据，未上场以及罚下血量为0
  */
typedef  struct {
	uint16_t red_1_robot_HP;
	uint16_t red_2_robot_HP;
	uint16_t red_3_robot_HP;
	uint16_t red_4_robot_HP;
	uint16_t red_5_robot_HP;
	uint16_t red_7_robot_HP;
	uint16_t red_outpost_HP;
	uint16_t red_base_HP;
	uint16_t blue_1_robot_HP;
	uint16_t blue_2_robot_HP;
	uint16_t blue_3_robot_HP;
	uint16_t blue_4_robot_HP;
	uint16_t blue_5_robot_HP;
	uint16_t blue_7_robot_HP;
	uint16_t blue_outpost_HP;
	uint16_t blue_base_HP;
} ext_game_robot_HP_t;



/** 人工智能挑战赛加成与惩罚区状态：0x0005   发送频率：1Hz周期发送    发送范围：所有机器人 */
typedef  struct {
	uint8_t F1_zone_status: 1;
	uint8_t F1_zone_buff_debuff_status: 3;
	uint8_t F2_zone_status: 1;
	uint8_t F2_zone_buff_debuff_status: 3;
	uint8_t F3_zone_status: 1;
	uint8_t F3_zone_buff_debuff_status: 3;
	uint8_t F4_zone_status: 1;
	uint8_t F4_zone_buff_debuff_status: 3;
	uint8_t F5_zone_status: 1;
	uint8_t F5_zone_buff_debuff_status: 3;
	uint8_t F6_zone_status: 1;
	uint8_t F6_zone_buff_debuff_status: 3;

	uint16_t red1_bullet_left;
	uint16_t red2_bullet_left;
	uint16_t blue1_bullet_left;
	uint16_t blue2_bullet_left;
} ext_ICRA_buff_debuff_zone_status_t;



/** 场地事件数据：0x0101    发送频率：事件改变后发送
  *	@param	bit 0-2：	己方补血点状态
  * 			bit 0：	己方补给站1号补血点占领状态，1为已占领
  *				bit 1：	己方补给站2号补血点占领状态，1为已占领
  *				bit 2：	己方补给站3号补血点占领状态，1为已占领
  *			bit 3-5：	己方能量机关状态
  *				bit 3：	打击点占领状态，1为占领
  *				bit 4：	小能量机关激活状态，1为已激活
  *				bit 5：	大能量机关激活状态，1为已激活
  *			bit 6-11：	己方高地占领状态
  *			    bit 6-7：   己方环形高地占领状态，1为己方占领，2为对方占领
  *			    bit 8-9：   己方R3梯高占领状态，1为己方占领，2为对方占领
  *			    bit 10-11： 己方R4梯高占领状态，1为己方占领，2为对方占领
  *			bit 12-18：	己方基地虚拟护盾的剩余值百分比
  *			bit 19-27：	飞镖最后一次击中己方前哨站或基地的时间(0-420)
  *			bit 28-29:  飞镖最后一次击中己方前哨站或基地的具体目标
  *			bit 30-31：  中心增益点的占领情况

  */
typedef  struct {
	uint32_t event_type;
} ext_event_data_t;



/** 补给站动作标识：0x0102   发送频率：动作改变后发送    发生范围：己方机器人
  * @param	supply_projectile_id	补给站剩余弹量:	1为1号补给口 2为2号补给口
  *			supply_robot_id			补弹机器人ID: 0为当前无机器人补弹
  *			supply_projectile_step	出弹口开闭状态：0为关闭，1为子弹准备中，2为子弹下落
  *			supply_projectile_num	补弹数量：50/100/150/200
  */
typedef  struct {
	uint8_t reserved;
	uint8_t supply_robot_id;
	uint8_t supply_projectile_step;
	uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;



/** 裁判警告信息：0x0104    发送频率：警告发生后发送
  *	@param	level	警告等级：1为黄牌，2为红牌，3为判负
  *			offending_robot_id	犯规机器人ID
  *			count   违规机器人对应判罚等级的违规次数
  */
typedef  struct {
	uint8_t level;
    uint8_t offending_robot_id;
    uint8_t count;
} ext_referee_warning_t;



/** 飞镖发射口倒计时：0x0105  发射频率：1Hz    发送范围：己方机器人 
  *	@param	dart_remaining_time		15s倒计时
  */
typedef  struct {
	uint8_t dart_remaining_time;
} ext_dart_remaining_time_t;



/** 比赛机器人状态：0x0201   发送频率：10Hz 
  *	@param	robot_id	本机器人ID
  *			robot_level	机器人等级
  *			remain_HP	机器人剩余血量
  *			max_HP		机器人上限血量
  *			shooter_barrel_cooling_value	机器人枪口热量每秒冷却值
  *			shooter_barrel_heat_limit	    机器人枪口热量上限
  *			chassis_power_limit	            机器人底盘功率上限
  *			主控电源输出情况
  *			mains_power_gimbal_output	gimbal口输出	1为有24V输出，0为无24V输出
  *			mains_power_chassis_output	chassis口输出	1为有24V输出，0为无24V输出
  *			mains_power_shooter_output	shooter口输出	1为有24V输出，0为无24V输出
  */
typedef  struct {
	uint8_t robot_id;
	uint8_t robot_level;
	uint16_t remain_HP;
	uint16_t max_HP;
	uint16_t shooter_barrel_cooling_value;
	uint16_t shooter_barrel_heat_limit;
	uint16_t chassis_power_limit;

	uint8_t mains_power_gimbal_output: 1;
	uint8_t mains_power_chassis_output: 1;
	uint8_t mains_power_shooter_output: 1;
} ext_game_robot_status_t;



/** 实时功率热量数据：0x0202  发送频率：50Hz 
  * @param	chassis_volt	底盘输出电压 单位mV
  *			chassis_current	底盘输出电流 单位mA
  *			chassis_power	底盘输出功率 单位W
  *			chassis_power_buffer			底盘功率缓冲 单位J	备注：飞坡根据规则增加至250J
  *			shooter_id1_17mm_cooling_heat	1号17mm枪口热量
  *			shooter_id2_17mm_cooling_heat	2号17mm枪口热量
  *			shooter_id1_42mm_cooling_heat	42mm枪口热量
  */
typedef  struct {
	uint16_t chassis_volt;
	uint16_t chassis_current;
	float chassis_power;
	uint16_t chassis_power_buffer;
	uint16_t shooter_id1_17mm_cooling_heat;
	uint16_t shooter_id2_17mm_cooling_heat;
	uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;



/** 机器人位置：0x0203     发送频率：10Hz
  *	@param	x	位置x坐标，单位m
  *			y	位置y坐标，单位m
  *			yaw	位置枪口，单位°
  */
typedef  struct {
    float x;
    float y;
    float yaw;
} ext_game_robot_pos_t;



/** 机器人增益：0x0204     发送频率：1Hz
  *	@param	bit 0:	机器人血量补血增益
  *			bit 1:	枪口热量冷却加速
  *			bit 2:	机器人防御加成
  *			bit 3:	机器人负防御加成
  *			bit 4:	机器人攻击加成
  */
typedef  struct {
    uint8_t hp_buff;
    uint8_t cooling_buff;
    uint8_t defence_buff;
    uint8_t vulnerability_buff;
    uint16_t attack_buff;
} ext_buff_t;



/** 空中机器人能量状态：0x0205     发送频率：10Hz
  *	@param	attack_time	可攻击时间 单位s	30s递减至0
  */
typedef  struct {
	uint8_t attack_time;
} aerial_robot_energy_t;

/** 伤害状态：0x0206          发送频率：伤害发生后发送
  *	@param	armor_id	当血量变化类型为装甲伤害，代表装甲ID，其中数值为0~4号代表机器人的五个装甲片，
  *						其他血量变化类型，该变量数值为0
  *			hurt_type	血量变化类型
  *							0x00 装甲伤害扣血
  *							0x01 模块掉线扣血
  *							0x02 超射速扣血
  *							0x03 超枪口热量扣血
  *							0x04 超底盘功率扣血
  *							0x05 装甲撞击扣血
  */
typedef   struct {
	uint8_t armor_id: 4;
	uint8_t hurt_type: 4;
} ext_robot_hurt_t;



/** 实时射击信息：0x0207    发送频率：射击后发送
  *	@param	bullet_type 子弹类型	1为17mm弹丸	2为42mm弹丸
  *			shooter_id	发射机构ID	1为1号17mm发射机构
  *									2为2号17mm发射机构
  *									3为42mm发射机构
  *			bullet_freq		子弹射频 单位Hz
  *			bullet_speed 	子弹射速 单位m/s
  */
typedef   struct {
	uint8_t bullet_type;
	uint8_t shooter_id;
	uint8_t bullet_freq;
	float bullet_speed;
} ext_shoot_data_t;



/** 子弹剩余发射数：0x0208   发送频率：10Hz周期发送   发送范围：所有机器人发送
*	@param	bullet_remaining_num_17mm	17mm子弹剩余发射数目
*			bullet_remaining_num_42mm	42mm子弹剩余发射数目
*			coin_remaining_num			剩余金币数量
  */
typedef  struct {
	uint16_t bullet_remaining_num_17mm;
	uint16_t bullet_remaining_num_42mm;
	uint16_t coin_remaining_num;
} ext_bullet_remaining_t;



/** 机器人RFID状态：0x0209     发送频率：1Hz    发送范围：单一机器人
  *	@param	bit 0:	己方基地增益点
  *			bit 1:	己方环形高地增益点
  *			bit 2:	对方环形高地增益点
  *			bit 3:	己方 R3/B3 梯形高地增益点
  *			bit 4:	对方 R3/B3 梯形高地增益点
  *			bit 5:	己方 R4/B4 梯形高地增益点
  *			bit 6:	对方 R4/B4 梯形高地增益点
  *			bit 7:	己方能量机关激活点
  *			bit 12：己方前哨站增益点
  *			bit 13：己方补血点（检测到任一均视为激活）
  *			bit 14：己方哨兵巡逻区
  *			bit 15：对方哨兵巡逻区
  *			bit 17：对方大资源岛增益点
* @brief	RFID状态不完全代表对应的增益或处罚状态，例如敌方已占领的高地增益点，不能获取对应的增益效应
  */
typedef  struct {
	uint32_t rfid_status;
} ext_rfid_status_t;



/** 飞镖机器人客户端指令数据：0x020A  发送频率：10Hz   发送范围：单一机器人
  *	@param	dart_launch_opening_status	当前飞镖发射口状态	0为关闭	1为正在开启或者关闭中 2为已经开启
  *			dart_attack_target			飞镖的打击目标，默认为前哨站	1为前哨站 2为基地
  *			target_change_time			切换打击目标时的比赛剩余时间，单位s，从未切换默认0
  *			first_dart_speed			检测到的第一枚飞镖速度，单位0.1m/s/LSB，未检测是为0
  *			second_dart_speed			检测到的第二枚飞镖速度，单位0.1m/s/LSB，未检测是为0
  *			third_dart_speed			检测到的第三枚飞镖速度，单位0.1m/s/LSB，未检测是为0
  *			fourth_dart_speed			检测到的第四枚飞镖速度，单位0.1m/s/LSB，未检测是为0
  *			last_dart_launch_time		最近一次的发射飞镖的比赛剩余时间，单位s，初始值为0
  *			operate_launch_cmd_time		最近一次操作手确定发射指令时的比赛剩余时间，单位s，初始值0
  */
typedef  struct {
	uint8_t dart_launch_opening_status;
	uint8_t dart_attack_target;
	uint16_t target_change_time;
	uint8_t first_dart_speed;
	uint8_t second_dart_speed;
	uint8_t third_dart_speed;
	uint8_t fourth_dart_speed;
	uint16_t last_dart_launch_time;
	uint16_t operate_launch_cmd_time;
} ext_dart_client_cmd_t;


/**
 * 地面机器人位置数据:0x020B固定以1Hz 频率发送
 *
 */
typedef  struct {
    float hero_x;
    float hero_y;
    float engineer_x;
    float engineer_y;
    float standard_3_x;
    float standard_3_y;
    float standard_4_x;
    float standard_4_y;
    float standard_5_x;
    float standard_5_y;
}ext_robot_pos_t;

/**
 * 哨兵自主决策信息同步:0x020D，固定以1Hz 频率发送
 */
typedef  struct {
    uint32_t sentry_info;
} ext_sentry_decision_t;



/** 交互数据 机器人间通信：0x0301 */
typedef  struct {
    uint16_t data_cmd_id;
    uint16_t sender_ID;
    uint16_t receiver_ID;
    uint8_t data[Custom_Data_Length_MAX];
} robot_interactive_data_t;



/** 客户端删除图形 机器人间通信：0x0301 
  *	@param	data_cmd_id		0x0100
  *			operate_type	图形操作	0-空操作 1-删除图层 2-删除所有
  *			layer			图层数		0~9
  */
typedef  struct {
	uint8_t operate_type;
	uint8_t layer;
} ext_client_custom_graphic_delete_t;

/** 机器人操作指令数据：0x0303
 * target_position_x 目标 x 位置坐标，单位 m, 当 target_robot_ID为0的时候，该项为0
 * target_position_y 目标 y 位置坐标，单位 m, 当 target_robot_ID为0的时候，该项为0
 * target_position_z 目标 z 位置坐标，单位 m, 当 target_robot_ID为0的时候，该项为0
 * commd_keyboard 云台手按键信息，无按键则为 0
 * target_robot_ID 要作用的目标机器人 ID 当发送位置信息时，该项为 0
 */
typedef struct {
  float target_position_x;
  float target_position_y;
  uint8_t commd_keyboard;
  uint8_t target_robot_ID;
  uint16_t cmd_source;
} ext_robot_command_t;


/** 图形数据 
  *	@param	graphic_name	图形名		在删除，修改等操作中，作为客户端的索引
  *			operate_type	图形操作	0-空操作 	1-增加 		2-修改 		3-删除
  *			graphic_type	图形类型	0-直线 		1-矩形		2-整圆		3-椭圆
  *										4-圆弧		5-浮点数	6-整型数	7-字符
  *			layre			图层数		0~9
  *			color			颜色		0-红蓝主色	1-黄色		2-绿色		3-橙色
  *										4-紫红色	5-粉色		6-青色		7-黑色
  *										8-白色
  *			start_angle		起始角度，单位°，范围[0,360]
  *			end_angle		终止角度，单位°，范围[0,360]
  *			width			线宽
  *			start_x			起点x坐标
  *			start_y			起点y坐标
  *			radius			字体大小或者半径
  *			end_x			终点x坐标
  *			end_y			终点y坐标
  *	@table
  *		graphic_type		start_angle		end_angle		width		start_x		start_y		radius		end_x		end_y
  *		直线				空				空				线条宽度	起点x坐标	起点y坐标	空			终点x坐标	终点y坐标
  *		矩形				空				空				线条宽度	起点x坐标	起点y坐标	空			对顶角x坐标	对顶角y坐标
  *		正圆				空				空				线条宽度	圆心x坐标	圆心y坐标	半径		空			空
  *		椭圆				空				空				线条宽度	圆心x坐标	圆心y坐标	空			x半轴长度	y半轴长度
  *		圆弧				起始角度		终止角度		线条宽度	圆心x坐标	圆心y坐标	空			x半轴长度	y半轴长度
  *		浮点数				字体大小		小数位有效个数	线条宽度	起点x坐标	起点y坐标			32位浮点数，float
  *		整型数				字体大小		空				线条宽度	起点x坐标	起点y坐标			32位整型数，int32_t
  *		字符				字体大小		字符长度		线条宽度	起点x坐标	起点y坐标	空			空			空
  */
typedef  struct {
	uint8_t graphic_name[3];
	uint32_t operate_type: 3;
	uint32_t graphic_type: 3;
	uint32_t layre: 4;
	uint32_t color: 4;
	uint32_t start_angle: 9;
	uint32_t end_angle: 9;
	uint32_t width: 10;
	uint32_t start_x: 11;
	uint32_t start_y: 11;
	uint32_t radius: 10;
	uint32_t end_x: 11;
	uint32_t end_y: 11;
} graphic_data_struct_t;



/** 客户端绘制一个图形 */
//#define ClientCustomGraphicSingleID		0x0101
typedef  struct {
	graphic_data_struct_t grapic_data_struct;
} ext_client_graphic_single_t;



/** 客户端绘制二个图形 */
//#define ClientCustomGraphicDoubleID		0x0102
typedef  struct {
	graphic_data_struct_t grapic_data_struct[2];
} ext_client_custom_graphic_double_t;



/** 客户端绘制五个图形 */
//#define ClientCustomGraphicFiveID			0x0103
typedef  struct {
	graphic_data_struct_t grapic_data_struct[5];
} ext_client_custom_graphic_five_t;



/** 客户端绘制七个图形 */
//#define ClientCustomGraphicSevenID		0x0104
typedef  struct {
	graphic_data_struct_t grapic_data_struct[7];
} ext_client_custom_graphic_seven_t;



/** 客户端绘制字符 */
//#define ClientCustomCharacterID			0x0110
typedef  struct {
	graphic_data_struct_t grapic_data_struct;
	uint8_t data[30];
} ext_client_custom_character_t;

#ifdef __cplusplus
#pragma pack()
#endif

#endif
