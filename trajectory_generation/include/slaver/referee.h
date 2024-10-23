#pragma once
#include "rm_rs_v2023_types.h"

typedef unsigned char  		UCHAR8;                  /** defined for unsigned 8-bits integer variable 	    无符号8位整型变量       */
typedef signed   char  		SCHAR8;                  /** defined for signed 8-bits integer variable		    有符号8位整型变量       */
typedef unsigned short 		USHORT16;                /** defined for unsigned 16-bits integer variable 	    无符号16位整型变量      */
typedef signed   short 		SSHORT16;                /** defined for signed 16-bits integer variable 	    有符号16位整型变量      */
typedef unsigned int   		UINT32;                  /** defined for unsigned 32-bits integer variable 	    无符号32位整型变量      */
typedef int   				SINT32;                  /** defined for signed 32-bits integer variable         有符号32位整型变量      */
typedef float          		FP32;                    /** single precision floating point variable (32bits)   单精度浮点数（32位长度） */
typedef double         		DB64;                    /** double precision floating point variable (64bits)   双精度浮点数（64位长度） */
typedef UCHAR8            u8;                      /** defined for unsigned 8-bits integer variable 	        无符号8位整型变量  */
typedef USHORT16          u16;                     /** defined for unsigned 16-bits integer variable 	    无符号16位整型变量 */
typedef UINT32            u32;                     /** defined for unsigned 32-bits integer variable 	    无符号32位整型变量 */
typedef SCHAR8            s8;                      /** defined for unsigned 8-bits integer variable 	        无符号8位整型变量  */
typedef SSHORT16          s16;                     /** defined for unsigned 16-bits integer variable 	    无符号16位整型变量 */
typedef SINT32            s32;                     /** defined for unsigned 32-bits integer variable 	    无符号32位整型变量 */


class Referee {
public:
    /**裁判系统相关**/
    ext_game_status_t						GameStatus;						// 比赛状态数据
    ext_game_result_t						GameResult;						// 比赛结果数据
    ext_game_robot_HP_t						GameRobotHP;					// 机器人血量数据
    ext_ICRA_buff_debuff_zone_status_t		ICRABuffDebuffZoneStatus;		// 人工智能挑战赛加成与惩罚区状态
    ext_event_data_t						EventData;						// 场地事件数据
    ext_supply_projectile_action_t			SupplyProjectileAction;			// 补给站动作标识
    ext_referee_warning_t					RefereeWarning;					// 裁判警告信息
    ext_dart_remaining_time_t				DartRemainingTime;				// 飞镖发射口倒计时
    ext_game_robot_status_t					GameRobotStatus;				// 比赛机器人状态
    ext_power_heat_data_t					PowerHeatData;					// 实时功率热量数据
    ext_game_robot_pos_t					GameRobotPos;					// 机器人位置
    ext_buff_t								Buff;							// 机器人增益
    aerial_robot_energy_t					AerialRobotEnergy;				// 空中机器人能量状态
    ext_robot_hurt_t						RobotHurt = { .armor_id = 9, .hurt_type = 9};						// 伤害状态
    ext_shoot_data_t						ShootData;						// 实时射击信息
    ext_bullet_remaining_t					BulletRemaining;				// 子弹剩余发射数目
    ext_rfid_status_t						RFIDStatus;						// 机器人RFID状态
    ext_dart_client_cmd_t					DartClientCmd;					// 飞镖机器人客户端指令数据
//    ext_student_interactive_header_data_t	StudentInteractiveHeaderData;	// 机器人间交互信息
    robot_interactive_data_t				RobotInteractiveData;			// 交互数据
    ext_robot_command_t                     RobotCommandData;               // 云台手操作指令
    ext_sentry_decision_t                   SentryDecisionData;             // 烧饼决策数据同步
    ext_robot_pos_t                         RobotPosData;                   // 己方所有车的位置

    uint32_t GameStatus_cnt;                                       // 比赛状态数据
    uint32_t GameResult_cnt;                                       // 比赛结果数据
    uint32_t GameRobotHP_cnt;                                      // 机器人血量数据
    uint32_t ICRABuffDebuffZoneStatus_cnt;                     // 人工智能挑战赛加成与惩罚区状态
    uint32_t EventData_cnt;                                    // 场地事件数据
    uint32_t SupplyProjectileAction_cnt;              // 补给站动作标识
    uint32_t RefereeWarning_cnt;                      // 裁判警告信息
    uint32_t DartRemainingTime_cnt;                   // 飞镖发射口倒计时
    uint32_t GameRobotStatus_cnt;                     // 比赛机器人状态
    uint32_t PowerHeatData_cnt;                       // 实时功率热量数据
    uint32_t GameRobotPos_cnt;                                  // 机器人位置
    uint32_t Buff_cnt;                                                    // 机器人增益
    uint32_t AerialRobotEnergy_cnt;                                       // 空中机器人能量状态
    uint32_t RobotHurt_cnt;       // 伤害状态
    uint32_t ShootData_cnt;       // 实时射击信息
    uint32_t BulletRemaining_cnt; // 子弹剩余发射数目
    uint32_t RFIDStatus_cnt;                                       // 机器人RFID状态
    uint32_t DartClientCmd_cnt;                                    // 飞镖机器人客户端指令数据
//    uint32_t StudentInteractiveHeaderData_cnt; // 机器人间交互信息
    uint32_t RobotInteractiveData_cnt;         // 交互数据
    uint32_t RobotCommandData_cnt;             // 云台手操作数据
    uint32_t last_RobotCommandData_cnt;        // 云台手操作计时数据

    uint32_t SentryDecisionData_cnt;           // 烧饼决策数据同步
    uint32_t RobotPosData_cnt;
};