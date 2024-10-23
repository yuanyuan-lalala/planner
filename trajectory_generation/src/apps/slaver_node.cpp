#include <iostream>
#include <boost/thread.hpp>

#include "global.h"

int main(int argc, char **argv)
{
    /** 初始化 **/
    /* 注册节点 */
    ros::init(argc, argv, "slaver");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");


    /* 参数初始化 */
    Global::paramInit();

    /* 订阅器和发布器初始化 */
    Global::speed_sub = n.subscribe("/sentry_des_speed", 1, Global::rcvSpeedCallBack, ros::TransportHints().unreliable().reliable().tcpNoDelay());
    Global::sentry_location = n.subscribe("/odometry_imu", 1, Global::rcvLocationCallBack, ros::TransportHints().unreliable().reliable().tcpNoDelay());
//    Global::xtl_flag_sub = n.subscribe("xtlflag", 1, Global::rcvXtlFlagCallBack);
    Global::global_path_sub = n.subscribe("/target_result",1, Global::rcvTargetCallBack);  // 把目标位置发到下位机用于可视化
    Global::global_object_sub = n.subscribe("/objects",1, Global::rcvGlobalObjectsCallBack);

    Global::arrive_goal_pub = nh.advertise<nav_msgs::Path>("arrive_goal", 1);
    Global::wheel_state_pub = nh.advertise<geometry_msgs::Vector3>("wheel_state", 1);
    Global::detect_enemy_pub = n.advertise<sentry_msgs::DetectObject>("detect_object", 1);
    Global::perception_sub = n.subscribe("/object", 1, Global::rcvAttackCallBack);
    Global::buff_status_pub = nh.advertise<sentry_msgs::BuffStatus>("buff_status", 1);
    Global::damage_feedback_pub = nh.advertise<sentry_msgs::DamageFeedback>("damage_feedback", 1);
    Global::eco_status_pub = nh.advertise<sentry_msgs::EcoStatus>("eco_status", 1);
    Global::game_status_pub = nh.advertise<sentry_msgs::GameStatus>("game_status", 1);
    Global::referee_warn_pub = nh.advertise<sentry_msgs::RefereeWarn>("referee_warn", 1);
    Global::robot_buff_pub = nh.advertise<sentry_msgs::RobotBuff>("robot_buff", 1);
    Global::robot_RFID_pub = nh.advertise<sentry_msgs::RobotRFID>("robot_RFID", 1);
    Global::robot_HP_pub = nh.advertise<sentry_msgs::RobotsHP>("robot_HP", 1);
    Global::robot_status_pub = nh.advertise<sentry_msgs::RobotStatus>("robot_status", 1);
    Global::shoot_feedback_pub = nh.advertise<sentry_msgs::ShootFeedback>("shoot_feedback", 1);
    Global::user_interaction_pub = nh.advertise<sentry_msgs::UserInteraction>("user_interaction", 1);
    Global::UWB_location_pub = nh.advertise<sentry_msgs::UWBLocation>("UWB_location", 1);
    Global::user_interaction_arrival_goal_pub = nh.advertise<sentry_msgs::UserInteraction_arrival_goal>("UserInteraction_arrival_goal", 1);
    Global::robot_command_pub = nh.advertise<sentry_msgs::RobotCommand>("robot_command", 1);
    Global::robot_pos_pub = nh.advertise<sentry_msgs::RobotPos>("robot_pos", 1);
//    Global::xtl_ready_flag_pub = nh.advertise<std_msgs::Bool>("ready_xtl_flag", 1);
    /* 绑定串口 */
    Global::slaver_serial = std::make_unique<SlaverSerial>("auto", 921600);

    /* 开启串口接收线程 */
    auto serialThread = boost::thread([&]
                                      { Global::slaver_serial->receive(Global::slaver_serial_data); });
    serialThread.detach();

    /* 1ms处理一次订阅回调函数 */
    ros::Rate rate(200);
    while (ros::ok())
    {
//        ROS_WARN("start slaver");

        ros::spinOnce();
        // /* 发送期望速度到下位机 */
        // std::vector<float> float_vec;
        // std::vector<unsigned char> char_vec;
        // float_vec.push_back(1.12);
        // float_vec.push_back(1.34);
        // float_vec.push_back(5.0);
        // float_vec.push_back(Global::enemy_x);
        // float_vec.push_back(Global::enemy_y);
        // float_vec.push_back(Global::enemy_z);
        // float_vec.push_back(Global::enemy_value);
        // char_vec.push_back(Global::enemy_id);
        // Global::slaver_serial->send(float_vec, char_vec);
        // static int times = 0;
        // ROS_WARN("%d th sentry command (a_t, a_c, s_t) = (%.2%, %.2f, %.2f)\n", times++, float_vec[0], float_vec[1], float_vec[2]);
        rate.sleep();
    }
}
