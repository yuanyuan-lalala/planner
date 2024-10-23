#include "uart_protocol.h"
#include "rm_rs_v2023_types.h"
#include "verify.h"
#include "referee.h"
#include "global.h"

sentry_msgs::GameStatus game_status;
sentry_msgs::RobotsHP robot_HP;
sentry_msgs::BuffStatus buff_status;
sentry_msgs::RefereeWarn referee_warn;
sentry_msgs::RobotStatus robot_status;
sentry_msgs::UWBLocation UWB_location;
sentry_msgs::RobotBuff robot_buff;
sentry_msgs::DamageFeedback damage_feedback;
sentry_msgs::ShootFeedback shoot_feedback;
sentry_msgs::EcoStatus eco_status;
sentry_msgs::RobotRFID robot_RFID;
sentry_msgs::UserInteraction user_interaction;
sentry_msgs::RobotCommand robot_command;
sentry_msgs::SentryInfo sentry_info;
sentry_msgs::RobotPos robot_pos;
bool robot_command_change;

static void MatchRSData(u16 cmdID, u8 *pData, u16 Size);

bool decode_num(unsigned char buff, float &num)
{
    static float2uchar temp_num;  ///转换结构
    static int pos = 0;           ///读取计数，每到 4 个为一组数据
    temp_num.ch[pos] = buff;
    pos++;
    if (pos == 4) {
        num = temp_num.f;
        pos = 0;
        return true;
    }
    return false;
}

/*接收到的来自裁判系统的字节在数据包中位置*/
typedef enum {
	RS_RX_FREE = 0,
	RS_RX_Length = 1,
	RS_RX_Num = 2,
	RS_RX_CRC8 = 3,
	RS_RX_CmdID = 4,
	RS_RX_Data = 5,
	RS_RX_CRC16 = 6,
}RS_RX_Status;

/** --------------------------------------------------------------------------
  * @brief  裁判系统数据解码
  
  * @retval None
  
  * @param	uart_rx:	串口接收状态机结构体
			pData:		RoboMaster裁判系统信息存储数组
			Size:		接收数据长度

  * @note	解码规则详情参考裁判系统学生串口协议附录（RoboMaster官方文档）
			解码规则以当年裁判系统串口协议为首要依据，每年可能会有少许变化
			2022年：frame_header (5-byte，SOF, data_length, seq, CRC8) + cmd_id (2-byte) + data (n-byte) + frame_tail (2-byte，CRC16，整包校验)
 -------------------------------------------------------------------------- **/
void UART_RSDataDecode(u8* buffer, u16 Size)
{
	RS_RX_Status RXStatus = RS_RX_FREE;
	u8 BufNum = 0;
	u8 RsData;
	u16 RsDataLength;
	u8 CorrectFlag = 0;															// 正确接收标志位
	u8 pData[250] = {0};
	u16 RsCmdID;

	for (int i = 0; i < Size; i++)												// 遍历每个接收的字节
	{
		RsData = buffer[i];														// 第i个接收的字节
		switch (RXStatus)														// 判断该字节在数据包中所处的位置
		{
			case RS_RX_FREE:
				if (RsData == 0xA5)												// 数据包起始帧SOF固定值0xA5
				{
					BufNum = 0;													// 第0位数据
					RXStatus = RS_RX_Length;									// 起始帧校验成功准备解码下一个数据段：有效数据长度
					pData[BufNum++] = 0xA5;										// 把该位数据存入用户定义的接收数组中			
				}
				else															// 数据失真则直接放弃接下来的数据解码防止解码错误，等待下一次接收
				{
					RXStatus = RS_RX_FREE;
				}
				break;
			
			case RS_RX_Length:
				pData[BufNum++] = RsData;										// 接收表示数据长度的两个字节
				if(BufNum == 3)
				{
					RsDataLength = pData[1] | (pData[2] << 8);					// 得到有效数据长度（data_length占两个字节）
					RXStatus = RS_RX_Num;										// 准备接收包序号
				}
				break;
			
			case RS_RX_Num:
				pData[BufNum++] = RsData;										// 接收包序号
				RXStatus = RS_RX_CRC8;											// 准备接收CRC8校验字
				break;
			
			case RS_RX_CRC8:
				pData[BufNum++] = RsData;										// 接收CRC8校验字
				if (Verify_CRC8_Check_Sum(pData, BufNum))
				{	
					RXStatus = RS_RX_CmdID;										// CRC8校验成功，准备接收数据ID
				}
				else															
				{
					RXStatus = RS_RX_FREE;										// CRC8校验失败，放弃此次接收的数据
				}
				break;
			
			case RS_RX_CmdID:
				pData[BufNum++] = RsData;										// 接收数据ID
				if(BufNum == 7)
				{
					RsCmdID = pData[5] | (pData[6] << 8);						// 得到数据ID号
					RXStatus = RS_RX_Data;										// 准备接收数据	
				}
				break;
				
			case RS_RX_Data:
				pData[BufNum++] = RsData;										// 接收数据
				if(BufNum == 7 + RsDataLength)
				{
					RXStatus = RS_RX_CRC16;										// 准备接收两个CRC16校验字节	
				}
				break;

			case RS_RX_CRC16:
				pData[BufNum++] = RsData;										// 接收两个CRC16校验字节
				if(BufNum == 9 + RsDataLength)
				{
					if (Verify_CRC16_Check_Sum(pData, BufNum))
					{	
						MatchRSData(RsCmdID, pData, RsDataLength);				// 裁判系统数据复制到对应结构体	（注！！:一次空闲中断会接收到多组数据（一次中断会多次解码））
						RXStatus = RS_RX_FREE;
					}
				}
				break;

			default:
				break;
		}
	}
}



/**
  * @brief  裁判系统数据复制到对应结构体，协议为20210203 v1.0版本
  * @retval None
  * @param	cmdID:	裁判系统数据指令ID
  * 		pData:	RoboMaster裁判系统信息存储数组
  * 		Size:	数据长度
  */
void MatchRSData(u16 cmdID, u8 *pData, u16 Size)
{
	switch (cmdID)
	{
		case GameStatusID:
			memcpy(&Global::referee.GameStatus, &pData[7], Size);
			game_status.game_type = Global::referee.GameStatus.game_type;
			game_status.game_status = Global::referee.GameStatus.game_progress;
			game_status.remaining_time = Global::referee.GameStatus.stage_remain_time;
			game_status.sync_time_stamp = Global::referee.GameStatus.SyncTimeStamp;
			Global::game_status_pub.publish(game_status);
			Global::referee.GameStatus_cnt++;
			break;
		case GameRobotHPID:
			memcpy(&Global::referee.GameRobotHP, &pData[7], Size);
			robot_HP.red_1_hp = Global::referee.GameRobotHP.red_1_robot_HP;
			robot_HP.red_2_hp = Global::referee.GameRobotHP.red_2_robot_HP;
			robot_HP.red_3_hp = Global::referee.GameRobotHP.red_3_robot_HP;
			robot_HP.red_4_hp = Global::referee.GameRobotHP.red_4_robot_HP;
			robot_HP.red_5_hp = Global::referee.GameRobotHP.red_5_robot_HP;
			robot_HP.red_sentry_hp = Global::referee.GameRobotHP.red_7_robot_HP;
			robot_HP.red_outpost_hp = Global::referee.GameRobotHP.red_outpost_HP;
			robot_HP.red_base_hp = Global::referee.GameRobotHP.red_base_HP;
			robot_HP.blue_1_hp = Global::referee.GameRobotHP.blue_1_robot_HP;
			robot_HP.blue_2_hp = Global::referee.GameRobotHP.blue_2_robot_HP;
			robot_HP.blue_3_hp = Global::referee.GameRobotHP.blue_3_robot_HP;
			robot_HP.blue_4_hp = Global::referee.GameRobotHP.blue_4_robot_HP;
			robot_HP.blue_5_hp = Global::referee.GameRobotHP.blue_5_robot_HP;
			robot_HP.blue_sentry_hp = Global::referee.GameRobotHP.blue_7_robot_HP;
			robot_HP.blue_outpost_hp = Global::referee.GameRobotHP.blue_outpost_HP;
			robot_HP.blue_base_hp = Global::referee.GameRobotHP.blue_base_HP;
			Global::robot_HP_pub.publish(robot_HP);
			Global::referee.GameRobotHP_cnt++;
			break;
		case EventDataID:
			memcpy(&Global::referee.EventData, &pData[7], Size);
			buff_status.hp_buff_1 = (bool)((Global::referee.EventData.event_type && ((uint32_t)1 << 0)) >> 0);
			buff_status.hp_buff_2 = (bool)((Global::referee.EventData.event_type && ((uint32_t)1 << 1)) >> 1);
			buff_status.hp_buff_3 = (bool)((Global::referee.EventData.event_type && ((uint32_t)1 << 2)) >> 2);
			buff_status.pm_loaction = (bool)((Global::referee.EventData.event_type && ((uint32_t)1 << 3)) >> 3);
			buff_status.small_pm_buff = (bool)((Global::referee.EventData.event_type && ((uint32_t)1 << 4)) >> 4);
			buff_status.big_pm_buff = (bool)((Global::referee.EventData.event_type && ((uint32_t)1 << 5)) >> 5);
			buff_status.rb2_mate_occupied = (bool)((Global::referee.EventData.event_type && ((uint32_t)1 << 6)) >> 6);
            buff_status.rb2_enemy_occupied = (bool)((Global::referee.EventData.event_type && ((uint32_t)1 << 7)) >> 7);
			buff_status.rb3_mate_occupied = (bool)((Global::referee.EventData.event_type && ((uint32_t)1 << 8)) >> 8);
            buff_status.rb3_enemy_occupied = (bool)((Global::referee.EventData.event_type && ((uint32_t)1 << 9)) >> 9);
			buff_status.rb4_mate_occupied = (bool)((Global::referee.EventData.event_type && ((uint32_t)1 << 10)) >> 10);
            buff_status.rb4_enemy_occupied = (bool)((Global::referee.EventData.event_type && ((uint32_t)1 << 11)) >> 11);


			buff_status.virtual_shield_percent = (int)((Global::referee.EventData.event_type && (0x00000000000001111111000000000000)) >> 12);
			buff_status.darts_time = (int)((Global::referee.EventData.event_type && (0x00001111111110000000000000000000)) >> 19);
			buff_status.outpost_hit = (bool)((Global::referee.EventData.event_type && ((uint32_t)1 << 28)) >> 28);
            buff_status.base_hit = (bool)((Global::referee.EventData.event_type && ((uint32_t)1 << 29)) >> 29);
            if(buff_status.base_hit){
                buff_status.outpost_hit = false;
            }
            Global::buff_status_pub.publish(buff_status);
			Global::referee.EventData_cnt++;
			break;
		case RefereeWaringID:
			memcpy(&Global::referee.RefereeWarning, &pData[7], Size);
			referee_warn.warn = (bool)((Global::referee.RefereeWarning.level));
			referee_warn.robot_id = (bool)((Global::referee.RefereeWarning.offending_robot_id));
			Global::referee_warn_pub.publish(referee_warn);
			Global::referee.RefereeWarning_cnt++;
			break;
		case DartRemainingTimeID:
			memcpy(&Global::referee.DartRemainingTime, &pData[7], Size);
			break;
		case GameRobotStatusID:
			memcpy(&Global::referee.GameRobotStatus, &pData[7], Size);
			robot_status.id = Global::referee.GameRobotStatus.robot_id;
			robot_status.level = Global::referee.GameRobotStatus.robot_level;
			robot_status.remain_hp = Global::referee.GameRobotStatus.remain_HP;
			robot_status.max_hp = Global::referee.GameRobotStatus.max_HP;
			robot_status.shooter_barrel_cooling_value = Global::referee.GameRobotStatus.shooter_barrel_cooling_value;
			robot_status.shooter_barrel_heat_limit = Global::referee.GameRobotStatus.shooter_barrel_heat_limit;
			robot_status.chassis_power_limit = Global::referee.GameRobotStatus.chassis_power_limit;
			robot_status.gimbal_output = Global::referee.GameRobotStatus.mains_power_gimbal_output;
			robot_status.chassis_output = Global::referee.GameRobotStatus.mains_power_chassis_output;
			robot_status.shooter_output = Global::referee.GameRobotStatus.mains_power_shooter_output;
			Global::robot_status_pub.publish(robot_status);
			Global::referee.GameRobotStatus_cnt++;
			break;
		case PowerHeatDataID:
			memcpy(&Global::referee.PowerHeatData, &pData[7], Size);
			Global::referee.PowerHeatData_cnt++;
			break;
		case GameRobotPosID:
			memcpy(&Global::referee.GameRobotPos, &pData[7], Size);
			UWB_location.x =  Global::referee.GameRobotPos.x;
			UWB_location.y = Global::referee.GameRobotPos.y;
            UWB_location.z = 0.0;
			UWB_location.yaw = Global::referee.GameRobotPos.yaw;
			Global::UWB_location_pub.publish(UWB_location);
			Global::referee.GameRobotPos_cnt++;
			break;
		case BuffID:
			memcpy(&Global::referee.Buff, &pData[7], Size);
			robot_buff.hp_buff = (bool)(Global::referee.Buff.hp_buff);
			robot_buff.cool_buff = (bool)(Global::referee.Buff.cooling_buff);
			robot_buff.defense_buff = (bool)(Global::referee.Buff.defence_buff);
			robot_buff.damage_buff = (bool)(Global::referee.Buff.attack_buff);
            robot_buff.vulnerability_buff = (bool)(Global::referee.Buff.vulnerability_buff);
			Global::robot_buff_pub.publish(robot_buff);
			Global::referee.Buff_cnt++;
			break;
		case RobotHurtID:
			memcpy(&Global::referee.RobotHurt, &pData[7], Size);
			damage_feedback.damage_type = Global::referee.RobotHurt.hurt_type;
			damage_feedback.damage_source = Global::referee.RobotHurt.armor_id;
			Global::damage_feedback_pub.publish(damage_feedback);
			Global::referee.RobotHurt_cnt++;
			break;
		case ShootDataID:
			memcpy(&Global::referee.ShootData, &pData[7], Size);
			shoot_feedback.frequency = Global::referee.ShootData.bullet_freq;
			shoot_feedback.speed = Global::referee.ShootData.bullet_speed;
			Global::shoot_feedback_pub.publish(shoot_feedback);
			Global::referee.ShootData_cnt++;
			break;
		case BulletRemainingID:
			memcpy(&Global::referee.BulletRemaining, &pData[7], Size);
			eco_status.small_bullet_num = Global::referee.BulletRemaining.bullet_remaining_num_17mm;
			eco_status.big_bullet_num = Global::referee.BulletRemaining.bullet_remaining_num_42mm;
			eco_status.money_num = Global::referee.BulletRemaining.coin_remaining_num;
			Global::eco_status_pub.publish(eco_status);
			break;
		case RFIDStatusID:  // TODO 重改
			memcpy(&Global::referee.RFIDStatus, &pData[7], Size);
            robot_RFID.base_rfid = (bool)((Global::referee.RFIDStatus.rfid_status & ((uint32_t)1 << 0)) >> 0);
            ROS_ERROR("robot_RFID: %d", Global::referee.RFIDStatus.rfid_status);
            ROS_ERROR("robot_RFID.base_rfid nice: %d"  , robot_RFID.base_rfid);
            robot_RFID.highland_rfid_circle = (bool)((Global::referee.RFIDStatus.rfid_status & ((uint32_t)1 << 1)) >> 1);
            robot_RFID.highland_rfid_R3 = (bool)((Global::referee.RFIDStatus.rfid_status & ((uint32_t)1 << 3)) >> 3);
            robot_RFID.highland_rfid_R4 = (bool)((Global::referee.RFIDStatus.rfid_status & ((uint32_t)1 << 5)) >> 5);
            robot_RFID.outpost_rfid = (bool)((Global::referee.RFIDStatus.rfid_status & ((uint32_t)1 << 12)) >> 12);
            robot_RFID.sentry_rfid = (bool)((Global::referee.RFIDStatus.rfid_status & ((uint32_t)1 << 14)) >> 14);


            Global::robot_RFID_pub.publish(robot_RFID);
			Global::referee.RFIDStatus_cnt++;
			break;
        case SentryDecisionID:
            memcpy(&Global::referee.SentryDecisionData, &pData[7], Size);
            sentry_info.exchange_bullet_number = (int)((Global::referee.SentryDecisionData.sentry_info & 1025));
            sentry_info.exchange_bullet_number_remote = (int)((Global::referee.SentryDecisionData.sentry_info & 16385) >> 11);
            sentry_info.hp_exchange_num_remote = (int)((Global::referee.SentryDecisionData.sentry_info & 262145) >> 15);
            Global::referee.SentryDecisionData_cnt++;
            break;
        case RobotPosID:
            memcpy(&Global::referee.RobotPosData, &pData[7], Size);
            robot_pos.hero_x = Global::referee.RobotPosData.hero_x;
            robot_pos.hero_y = Global::referee.RobotPosData.hero_y;
            robot_pos.engineer_x = Global::referee.RobotPosData.engineer_x;
            robot_pos.engineer_y = Global::referee.RobotPosData.engineer_y;
            robot_pos.standard_3_x = Global::referee.RobotPosData.standard_3_x;
            robot_pos.standard_3_y = Global::referee.RobotPosData.standard_3_y;
            robot_pos.standard_4_x = Global::referee.RobotPosData.standard_4_x;
            robot_pos.standard_4_y = Global::referee.RobotPosData.standard_4_y;
            robot_pos.standard_5_x = Global::referee.RobotPosData.standard_5_x;
            robot_pos.standard_5_y = Global::referee.RobotPosData.standard_5_y;
            Global::robot_pos_pub.publish(robot_pos);
            break;

		case RobotInteractiveDataID:
		{
            ROS_ERROR("301 nice");
            memcpy(&Global::referee.RobotInteractiveData, &pData[7], Size);
            std::cout << sizeof(Global::referee.RobotInteractiveData.data) << std::endl;
            //	user_interaction.msg_id = Global::referee.StudentInteractiveHeaderData.data_cmd_id;
            std::cout << "data_cmd_id: " << (int)Global::referee.RobotInteractiveData.data_cmd_id
                      << std::endl;
            std::cout << "receiver_ID: " << (int)Global::referee.RobotInteractiveData.receiver_ID
                      << std::endl;
            std::cout << "sender_ID: " << (int)Global::referee.RobotInteractiveData.sender_ID << std::endl;
            float temp;
            float tmpNum[28];
            int tmpj = 0;
            for (int i = 0; i < Size - 6; i++) {
                if (decode_num(Global::referee.RobotInteractiveData.data[i], temp)) {
                    tmpNum[tmpj] = temp;
                    tmpj++;
                    std::cout << "i: " << i << " 301 temp: " << temp << std::endl;
                }
            }
            user_interaction.dart_flag = tmpNum[0];
            for (int i = 0; i < 6; i++) {
                user_interaction.id[i] = tmpNum[1 + 4 * (i) + 0];
                user_interaction.pos_3d[i].x = tmpNum[1 + 4 * (i) + 1];
                user_interaction.pos_3d[i].y = tmpNum[1 + 4 * (i) + 2];
                user_interaction.pos_3d[i].z = tmpNum[1 + 4 * (i) + 3];
            }
            Global::user_interaction_pub.publish(user_interaction);
            Global::referee.RobotInteractiveData_cnt++;
            break;
		}
		case RobotCommandID:

			memcpy(&Global::referee.RobotCommandData, &pData[7], Size);
			robot_command.target_position_x = Global::referee.RobotCommandData.target_position_x;
			robot_command.target_position_y = Global::referee.RobotCommandData.target_position_y;
			robot_command.target_position_z = 0.0;
			robot_command.commd_keyboard = Global::referee.RobotCommandData.commd_keyboard;

			robot_command.target_robot_ID = Global::referee.RobotCommandData.target_robot_ID;
            ROS_WARN("robot command: %d", robot_command.commd_keyboard);
            ROS_WARN("robot command target_robot_ID: %d", robot_command.target_robot_ID);

            Global::robot_command_pub.publish(robot_command);
			Global::referee.RobotCommandData_cnt++;
			break;
		default:
			break;
	}
}

