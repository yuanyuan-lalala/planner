#pragma once

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Path.h>
#include "planner/slaver_speed.h"

#include "SlaverSerial.h"
#include <sentry_msgs/object.h>
#include <sentry_msgs/objects.h>
#include "referee.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>


#include "sentry_msgs/BuffStatus.h"
#include "sentry_msgs/DamageFeedback.h"
#include "sentry_msgs/EcoStatus.h"
#include "sentry_msgs/GameStatus.h"
#include "sentry_msgs/RefereeWarn.h"
#include "sentry_msgs/RobotBuff.h"
#include "sentry_msgs/RobotRFID.h"
#include "sentry_msgs/RobotsHP.h"
#include "sentry_msgs/RobotStatus.h"
#include "sentry_msgs/ShootFeedback.h"
#include "sentry_msgs/UserInteraction.h"
#include "sentry_msgs/UWBLocation.h"
#include "sentry_msgs/UserInteraction_arrival_goal.h"
#include "sentry_msgs/RobotCommand.h"
#include "sentry_msgs/DetectObject.h"
#include "sentry_msgs/SentryInfo.h"
#include "sentry_msgs/RobotPos.h"


class Global {
public:
    /* 枚举类型：敌军颜色 */
    enum ENEMY_COLOR {
        BLUE,
        RED,
    };

    /* 串口相关 */
    static std::unique_ptr<SlaverSerial> slaver_serial;
    static std::vector<float> slaver_serial_data;
    static SlaverSerial::DATA_TYPE slaver_serial_data_type;

    /* ROS相关 */
    static ros::Subscriber speed_sub;   // 速度订阅器
    static ros::Subscriber sentry_location;
    static ros::Subscriber xtl_flag_sub;  // 小陀螺订阅
    static ros::Subscriber attack_engineer_sub; // 攻击工程车订阅
    static ros::Subscriber global_path_sub; // 目标点可视化
    static ros::Subscriber global_object_sub; // 给雷达发送的数据


    static ros::Publisher arrive_goal_pub;// 目标位置发布器
    static ros::Publisher user_interaction_arrival_goal_pub;
    static ros::Publisher wheel_state_pub;
    static ros::Publisher detect_enemy_pub;
    static ros::Subscriber perception_sub;// 全局视野目标订阅器
    static ros::Publisher buff_status_pub;
    static ros::Publisher damage_feedback_pub;
    static ros::Publisher eco_status_pub;
    static ros::Publisher game_status_pub;
    static ros::Publisher referee_warn_pub;
    static ros::Publisher robot_buff_pub;
    static ros::Publisher robot_RFID_pub;
    static ros::Publisher robot_HP_pub;
    static ros::Publisher robot_status_pub;
    static ros::Publisher robot_pos_pub;
    static ros::Publisher shoot_feedback_pub;
    static ros::Publisher user_interaction_pub;
    static ros::Publisher UWB_location_pub;
    static ros::Publisher robot_command_pub;
    static ros::Publisher xtl_ready_flag_pub;

    /* 比赛相关 */
    static enum Global::ENEMY_COLOR enemy_color; // 敌方颜色
    static uint8_t enemy_id;
    static float enemy_x;
    static float enemy_y;
    static float enemy_z;
    static float enemy_value;
    static Referee referee;
    static float sentry_location_x;
    static float sentry_location_y;
    static float target_location_x;  // 可视化目标位置
    static float target_location_y;

    static float angle_target;
    static float angle_current;
    static float line_speed;

    static uint8_t attack_engineer_flag;
    static uint8_t xtl_flag;
    static uint8_t in_bridge;

    static std::vector<float> global_objects;
    static std::vector<uint8_t> global_id;

    /* 全局初始化 */
    static void paramInit(void);

    /* 回调函数 */
    static void rcvSpeedCallBack(const planner::slaver_speed & speed);
    static void rcvAttackCallBack(const sentry_msgs::object & enemy);
    static void rcvLocationCallBack(const nav_msgs::Odometry &state);
    static void rcvXtlFlagCallBack(const std_msgs::Bool &msg);
    static void rcvTargetCallBack(const geometry_msgs::Point &path);
    static void rcvGlobalObjectsCallBack(const sentry_msgs::objects &msg);

};