#pragma once
#include"memory"
#include"ros/ros.h"
#include"planner_manager.hpp"
#include "map/global_map.hpp"
#include"visualizer/visualization.hpp"
#include <nav_msgs/Odometry.h>
#include "nav_msgs/Path.h"
#include <gazebo_msgs/ModelStates.h>
#include "std_msgs/Bool.h"
#include"std_msgs/Int64.h"
#include <sentry_msgs/object.h>
#include <sentry_msgs/objects.h>
#include <sentry_msgs/RobotsHP.h>
#include <sentry_msgs/RobotCommand.h>
#include <sentry_msgs/RobotStatus.h>
#include <sentry_msgs/GoTarget.h>
#include "planner/trajectoryPoly.h"



class ReplanFSM{
    public:
    typedef enum class FSM_EXEC_STATE{
        INIT,
        WAIT_TARGET,
        GEN_NEW_TRAJ,
        REPLAN_TRAJ,
        EXEC_TRAJ,
        WAIT_TOPO_TRAJ
    } planningStatus;

    typedef enum class TARGET_TYPE{
        MANUAL_TARGET = 1,
        PRESET_TARGET = 2,//预先设好的目标
        REFENCE_PATH = 3//参考路径
    } targetType;

    typedef enum class PLANNING_TYPE
    {
        TEAMFLIGHT = 1,//团队飞行
        SAFETEAMFLIGHT = 2,//安全
        FASTMOTION = 3//快速
    } planningType;

    planningStatus sentryStatus;
    targetType sentryTargetType;
    planningType sentryPlanningType;


    // enum class teamColor{
    //     RED,
    //     BLUE,
    // };
    teamColor sentryColor;
    

    void init(ros::NodeHandle &nh);

    void rcvWaypointsCallback(const nav_msgs::PathConstPtr &wp);  /// 接受手动指定点
    void rcvGazeboRealPosCallback(const gazebo_msgs::ModelStatesConstPtr &state);
    void rcvLidarIMUPosCallback(const nav_msgs::OdometryConstPtr &state);

 
    void rcvDebugFlag(const std_msgs::BoolConstPtr &msg);  /// 调试服务
    void rcvSentryWheelSpeedYawCallback(const geometry_msgs::Vector3ConstPtr &state);
    void rcvSentryEnemyXtlControlCallback(const sentry_msgs::objectsConstPtr &msg);
    void rcvSentryStatusCallback(const sentry_msgs::RobotStatusConstPtr &msg);
    void rcvPlanningModeCallback(const std_msgs::Int64ConstPtr &msg);
    void rcvSentryHPJudgeCallback(const sentry_msgs::RobotsHPConstPtr &msg);
    void rcvTargetPointsCallback(const geometry_msgs::PointConstPtr &point);
    void rcvRobotHPCallback(const sentry_msgs::RobotsHPConstPtr &msg);

    void execFSMCallback(const ros::TimerEvent &e);
    void execPlanningCallback(const ros::TimerEvent &e);
    void checkReplanCallback(const std_msgs::BoolConstPtr &msg);

    bool srvGoTargetCallBack(sentry_msgs::GoTarget::Request &target,
                                    sentry_msgs::GoTarget::Response &res);  /// 决策目标点

    bool have_target, have_odom, have_attack_target;


    std::unique_ptr<PlannerManager> plannerManager;
    std::unique_ptr<Visualization> visualization;
    std::shared_ptr<GlobalMap> m_global_map;

    double robot_radius;
    double robot_radius_dash;
    int decision_mode = 0;

    Eigen::Vector3d start_pt;
    Eigen::Vector3d final_goal;

    Eigen::Vector3d robot_cur_position;
    double robot_cur_yaw;

    Eigen::Vector3d attack_target_pos;
    Eigen::Vector3d attack_target_speed;

    Eigen::Quaterniond robot_cur_orientation;
    Eigen::Vector3d robot_cur_speed;
    double robot_wheel_speed;
    double robot_wheel_yaw;
    
    bool visualization_flag;
    bool replan_flag = false;  // 判断重规划是否成功
    bool planning_succeed;

    ros::Timer exeTimer,planningTimer,decisionTimer;
    
    ros::Publisher poly_traj_pub, grid_map_vis_pub, optimized_path_vis_pub, cur_position_vis_pub;
    ros::Publisher global_planning_result_pub, reference_path_vis_pub, attack_engineer_pub, local_grid_map_vis_pub;
    ros::Publisher spinning_mode_pub, target_point_pub;

    ros::Subscriber odom_sub, odom_gazebo_sub, slaver_sub, debug_sub, rviz_despt_sub;
    ros::Subscriber sentry_HP_sub, sentry_status_sub, robot_command_sub, enemy_num_sub;
    ros::Subscriber replan_flag_sub;
    ros::Subscriber planning_mode_sub, target_points_sub, robot_hp_sub, target_sim_sub;

    ros::ServiceServer go_target_server;
    

    inline double distance(Eigen::Vector3d target_pt, Eigen::Vector3d last_target_pt)
    {
        double dis = sqrt(pow(target_pt.x() - last_target_pt.x(), 2) + pow(target_pt.y() - last_target_pt.y(), 2));
        return dis;
    }
    

    private:







};
