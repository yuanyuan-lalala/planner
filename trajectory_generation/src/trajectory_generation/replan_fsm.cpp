#include"replan_fsm.hpp"

void ReplanFSM::init(ros::NodeHandle& nh){
    
    sentryStatus = planningStatus::INIT;
    sentryPlanningType = planningType::FASTMOTION;

    plannerManager.reset(new PlannerManager);
    plannerManager->init(nh);

    visualization.reset(new Visualization);
    visualization->init(nh);

    nh.param("trajectory_generator/robot_radius_dash", robot_radius_dash, 0.1);
    nh.param("trajectory_generator/robot_radius", robot_radius, 0.35);
    
    exeTimer = nh.createTimer(ros::Duration(0.03), &ReplanFSM::execFSMCallback, this);
    planningTimer = nh.createTimer(ros::Duration(0.03), &ReplanFSM::execPlanningCallback, this);

    replan_flag_sub = nh.subscribe("/replan_flag", 1, &ReplanFSM::checkReplanCallback, this);

    odom_sub = nh.subscribe("/odometry_imu", 1, &ReplanFSM::rcvLidarIMUPosCallback, this, ros::TransportHints().unreliable().reliable().tcpNoDelay());
    odom_gazebo_sub = nh.subscribe("/gazebo/model_states", 1, &ReplanFSM::rcvGazeboRealPosCallback, this);

    slaver_sub = nh.subscribe("/slaver/wheel_state", 1, &ReplanFSM::rcvSentryWheelSpeedYawCallback, this);
    rviz_despt_sub = nh.subscribe("/waypoint_generator/waypoints", 1, &ReplanFSM::rcvWaypointsCallback, this);
    debug_sub = nh.subscribe("/local_debug_flag", 1, &ReplanFSM::rcvDebugFlag, this);

    target_points_sub = nh.subscribe("/mbot/target_point", 1, &ReplanFSM::rcvTargetPointsCallback, this);
    go_target_server = nh.advertiseService("/GoTarget", &ReplanFSM::srvGoTargetCallBack, this);

    sentry_status_sub = nh.subscribe("/slaver/robot_status", 1, &ReplanFSM::rcvSentryStatusCallback, this);
    planning_mode_sub = nh.subscribe("/planning_mode", 1, &ReplanFSM::rcvPlanningModeCallback, this);
    robot_hp_sub = nh.subscribe("/slaver/robot_HP", 1, &ReplanFSM::rcvRobotHPCallback, this);

    target_point_pub = nh.advertise<geometry_msgs::Point>("/target_result", 1);
    global_planning_result_pub = nh.advertise<planner::trajectoryPoly>("global_trajectory", 1);
    spinning_mode_pub = nh.advertise<std_msgs::Int64>("/xtl_mode", 1);
    grid_map_vis_pub = nh.advertise<sensor_msgs::PointCloud2>("grid_map_vis", 1);
    local_grid_map_vis_pub = nh.advertise<sensor_msgs::PointCloud2>("local_grid_map_vis", 1);

}


void ReplanFSM::execFSMCallback(const ros::TimerEvent &e){
    geometry_msgs::Point target_vis;
    target_vis.x = final_goal.x();
    target_vis.y = final_goal.y();
    target_vis.z = final_goal.z();
    target_point_pub.publish(target_vis);
    
    plannerManager->m_astar_path_finder->visLocalGridMap(*(plannerManager->m_globalMap->m_local_cloud), true);
    
    visualization->visAstarPath(plannerManager->astar_path);
    visualization->visOptimizedPath(plannerManager->final_path);
    visualization->visOptGlobalPath(plannerManager->ref_trajectory);
    visualization->visFinalPath(plannerManager->optimized_path);
    //vislization->visObs(plannerManager->path_smoother->allobs);
   
    visualization->visCurPosition(robot_cur_position);
    visualization->visTopoPointGuard(plannerManager->m_topoPRM->m_graph);
    visualization->visTopoPointConnection(plannerManager->m_topoPRM->m_graph);
    visualization->visAttackPoint(attack_target_pos, attack_target_speed);
    
    std::vector<std::vector<Eigen::Vector3d>> temp_paths;
    if(plannerManager->local_optimize_path.size()> 0){
        temp_paths.push_back(plannerManager->local_optimize_path);
        //vislization->visTopoPath(temp_paths);
    }else{
        //vislization->visTopoPath(plannerManager->topo_prm->final_paths);
    }
    ros::Time t1, t2;  /// 计算时间
    t1 = ros::Time::now();
    if(sentryStatus==planningStatus::INIT){
        if(!have_odom){
            return;
        }
        sentryStatus = planningStatus::WAIT_TARGET;
    }
    if(sentryStatus == planningStatus::WAIT_TARGET){
        if(!have_target){
            return;
        }
        sentryStatus = planningStatus::GEN_NEW_TRAJ;
    }
    if(sentryStatus == planningStatus::GEN_NEW_TRAJ){
        if(plannerManager->pathFinding(start_pt, final_goal, robot_cur_speed)){
            sentryStatus = planningStatus::EXEC_TRAJ;
            planner::trajectoryPoly global_path;
            double desired_time = 0.0;
            global_path.motion_mode = decision_mode;
            for(int i = 0; i < plannerManager->m_referenceSmooth->m_trapezoidal_time.size(); i++){
                desired_time += plannerManager->m_referenceSmooth->m_trapezoidal_time[i];
//                global_path.order = 3;
                global_path.duration.push_back(plannerManager->m_referenceSmooth->m_trapezoidal_time[i]);
                global_path.coef_x.push_back(plannerManager->m_referenceSmooth->m_polyMatrix_x(i, 0)); // d
                global_path.coef_x.push_back(plannerManager->m_referenceSmooth->m_polyMatrix_x(i, 1));
                global_path.coef_x.push_back(plannerManager->m_referenceSmooth->m_polyMatrix_x(i, 2));
                global_path.coef_x.push_back(plannerManager->m_referenceSmooth->m_polyMatrix_x(i, 3));

                global_path.coef_y.push_back(plannerManager->m_referenceSmooth->m_polyMatrix_y(i, 0)); // d
                global_path.coef_y.push_back(plannerManager->m_referenceSmooth->m_polyMatrix_y(i, 1));
                global_path.coef_y.push_back(plannerManager->m_referenceSmooth->m_polyMatrix_y(i, 2));
                global_path.coef_y.push_back(plannerManager->m_referenceSmooth->m_polyMatrix_y(i, 3));
            }
            global_planning_result_pub.publish(global_path);
            ROS_INFO("[FSM] trajectory generate time: %f", (ros::Time::now() - t1).toSec());
            ROS_INFO("[FSM] trajectory desired time: %f", desired_time);
        }
        else
        {
            ROS_ERROR("[FSM] generate trajectory failed");
            return;
        }
    }



}
    
void ReplanFSM::execPlanningCallback(const ros::TimerEvent &e){
    std_msgs::Int64 spinningMode;
    if(plannerManager->is_spinning){
        spinningMode.data = 1;
    }else{
        spinningMode.data = 3;
    }
    spinning_mode_pub.publish(spinningMode);
}



void ReplanFSM::rcvLidarIMUPosCallback(const nav_msgs::OdometryConstPtr &state)
{
    /* 实时位置 */
    geometry_msgs::Pose pose;
    pose = state->pose.pose;
    robot_cur_position(0) = pose.position.x;
    robot_cur_position(1) = pose.position.y;
    robot_cur_position(2) = pose.position.z - 0.2;

    // ROS_INFO("Robot (X, Y) = (%f, %f)", pose.position.x, pose.position.y);

    /* 使用转换公式获取实时姿态(yaw取-pi~pi) 这里的yaw轴姿态是雷达姿态，这里因为雷达固连在底盘上所以雷达的姿态就等于底盘姿态 */
    double x = pose.orientation.x;
    double y = pose.orientation.y;
    double z = pose.orientation.z;
    double w = pose.orientation.w;
    // 航向角 yaw 的正弦分量。
    double siny_cosp = +2.0 * (w * z + x * y);
    // 航向角 yaw 的余弦分量。
    double cosy_cosp = +1.0 - 2.0 * (y * y + z * z);

    /* 重设规划起始位置 */
    start_pt(0) = robot_cur_position(0);
    start_pt(1) = robot_cur_position(1);
    start_pt(2) = robot_cur_position(2);

    Eigen::Vector3i start_idx = plannerManager->m_globalMap->coord2gridIndex(start_pt);
    if (plannerManager->m_globalMap->isOccupied(start_idx, false)) {
        Eigen::Vector3i start_neigh_idx;
        if (plannerManager->m_astar_path_finder->findNeighPoint(start_idx, start_neigh_idx, 2)) {
            start_idx = start_neigh_idx;
        }
    }
    Eigen::Vector3d start_temp = plannerManager->m_globalMap->gridIndex2coord(start_idx);
    start_pt.x() = start_temp.x();
    start_pt.y() = start_temp.y();  /// 保留初始z轴高度

    have_odom = true;
}


void ReplanFSM::rcvGazeboRealPosCallback(const gazebo_msgs::ModelStatesConstPtr &state)
{
    /* 实时位置 */
    //ROS_WARN("Robot (X, Y) = (%f, %f)", state->pose[1].position.x, state->pose[1].position.y);
    int robot_namespace_id = 0;
    for(int i = 0;i < state->name.size();i++){
        if(state->name[i] == "mbot"){
            robot_namespace_id = i;
            break;
        }
    }

    robot_cur_position(0) = state->pose[robot_namespace_id].position.x;
    robot_cur_position(1) = state->pose[robot_namespace_id].position.y;
    robot_cur_position(2) = state->pose[robot_namespace_id].position.z;

    /* 使用转换公式获取实时姿态(yaw取-pi~pi) */
    double x = state->pose[robot_namespace_id].orientation.x;
    double y = state->pose[robot_namespace_id].orientation.y;
    double z = state->pose[robot_namespace_id].orientation.z;
    double w = state->pose[robot_namespace_id].orientation.w;
    double siny_cosp = +2.0 * (w * z + x * y);
    double cosy_cosp = +1.0 - 2.0 * (y * y + z * z);
    robot_cur_yaw = atan2f(siny_cosp, cosy_cosp);

    /* 重设规划起始位置 */
    start_pt(0) = robot_cur_position(0);
    start_pt(1) = robot_cur_position(1);
    start_pt(2) = robot_cur_position(2);

    /* 将点云坐标映射为栅格ID(栅格ID从地图左下角算起) */
    Eigen::Vector3i start_idx = plannerManager->m_globalMap->coord2gridIndex(start_pt);
    if (plannerManager->m_globalMap->isOccupied(start_idx, false)) {
        Eigen::Vector3i start_neigh_idx;
        if (plannerManager->m_astar_path_finder->findNeighPoint(start_idx, start_neigh_idx, 2)) {
            start_idx = start_neigh_idx;
        }
    }
    Eigen::Vector3d start_temp = plannerManager->m_globalMap->gridIndex2coord(start_idx);
    start_pt.x() = start_temp.x();
    start_pt.y() = start_temp.y();  /// 保留初始z轴高度

    robot_cur_speed(0) = state->twist[robot_namespace_id].linear.x;
    robot_cur_speed(1) = state->twist[robot_namespace_id].linear.y;
    robot_cur_speed(2) = 0;
    have_odom = true;
}

void ReplanFSM::rcvSentryWheelSpeedYawCallback(const geometry_msgs::Vector3ConstPtr &state)
{
    // x: wheel_speed, y:wheel_yaw
    // 速度小则可以输入到车轮速度
    if(abs(state->x) < 10.0){
        robot_wheel_speed = state->x;
    }
    robot_wheel_yaw = state->y * M_PI / 180;
    int k = robot_wheel_yaw / (2 * M_PI);
    robot_wheel_yaw = robot_wheel_yaw - k * (2 * M_PI);
    //  -180° 到 180°
    int j = robot_wheel_yaw / M_PI;
    robot_wheel_yaw = robot_wheel_yaw - j * (2 * M_PI);

    if(have_odom){
        robot_cur_yaw = robot_wheel_yaw;
        robot_cur_speed(0) = robot_wheel_speed * cos(robot_wheel_yaw);
        robot_cur_speed(1) = robot_wheel_speed * sin(robot_wheel_yaw);
        robot_cur_speed(2) = 0;
    }
}

void ReplanFSM::rcvWaypointsCallback(const nav_msgs::PathConstPtr &wp)
{
    /* 读出三维目标值 */
    Eigen::Vector3d target_pt;
    target_pt << wp->poses[0].pose.position.x,
            wp->poses[0].pose.position.y,
            wp->poses[0].pose.position.z;

    target_pt.z() = 0;//z轴去掉，二维平面规划
    final_goal = target_pt;
    have_target = true;
    sentryStatus = planningStatus::GEN_NEW_TRAJ;
    ROS_INFO("[FSM Get Target] generate trajectory!]");
    std::cout<<"target_pt: "<<target_pt.x() <<", "<<target_pt.y()<<", "<<target_pt.z()<<std::endl;

    Eigen::Vector3i goal_idx = plannerManager->m_globalMap->coord2gridIndex(final_goal);
    if (plannerManager->m_globalMap->isOccupied(goal_idx, false)) {
        Eigen::Vector3i goal_neigh_idx;
        if (plannerManager->m_astar_path_finder->findNeighPoint(goal_idx, goal_neigh_idx, 2)) {
            goal_idx = goal_neigh_idx;
        }
    }
    final_goal = plannerManager->m_globalMap->gridIndex2coord(goal_idx);
    final_goal.z() = plannerManager->m_globalMap->getHeight(goal_idx.x(), goal_idx.y());
}

void ReplanFSM::rcvDebugFlag(const std_msgs::BoolConstPtr &msg)
{
    visualization_flag = msg->data;
    ROS_WARN("Visualization Flag Change");
}


void ReplanFSM::rcvSentryStatusCallback(const sentry_msgs::RobotStatusConstPtr &msg)
{
    if(msg->id == 7){
        // plannerManager->sentryColor = plannerManager->sentryColor
        plannerManager->sentryColor = plannerManager->teamColor::RED;
    }
    else if (msg->id == 107){
        plannerManager->sentryColor = plannerManager->teamColor::BLUE;
    }
    ROS_WARN("[FSM Status] sentry_color: %d",plannerManager->sentryColor);
}

void ReplanFSM::rcvPlanningModeCallback(const std_msgs::Int64ConstPtr &msg)
{
    sentryPlanningType = planningType(msg->data);
    
    ROS_WARN("planning mode change to %d", sentryPlanningType);

    if(sentryPlanningType == PLANNING_TYPE::TEAMFLIGHT){  // 团战模式
        plannerManager->reference_desire_speed = 3.0;
        plannerManager->reference_a_max = 9.0;
    }else if(sentryPlanningType == PLANNING_TYPE::SAFETEAMFLIGHT) {  // 虎步团战模式
        plannerManager->reference_desire_speed = 2.0;
        plannerManager->reference_a_max = 8.0;
    } else{
        plannerManager->reference_desire_speed = 3.6;
        plannerManager->reference_a_max = 7.0;
    }
}

void ReplanFSM::rcvRobotHPCallback(const sentry_msgs::RobotsHPConstPtr &msg) {
    double mate_outpost_hp = 1500;
    
    if (plannerManager->sentryColor == plannerManager->teamColor::RED) {
        mate_outpost_hp = msg->red_outpost_hp;
    } else {
        mate_outpost_hp = msg->blue_outpost_hp;
    }

    ROS_WARN("mate_outpost_hp: %f", mate_outpost_hp);

    if (mate_outpost_hp < 1) {
        plannerManager->is_spinning = true;  //  前哨站血量够低的话进小陀螺
    }else{
        plannerManager->is_spinning = false;
    }
};

void ReplanFSM::checkReplanCallback(const std_msgs::BoolConstPtr &msg)  //检查是否需要重规划
{
    if(plannerManager->optimized_path.size() < 2){
        return;
    }

    if(msg->data == true){

        ROS_ERROR("[FSM] replan trajectory now !");
        if(!replan_flag){
//            replan_flag = true;
            sentryStatus = planningStatus::REPLAN_TRAJ;
        }
    }else{
        replan_flag = false;
        return;
    }
}


void ReplanFSM::rcvTargetPointsCallback(const geometry_msgs::PointConstPtr &point)
{  //高频率接收目标点进行规划，用于高强度测试程序是否有bug以及会不会崩溃
    Eigen::Vector3d target_pt;
    target_pt << point->x,
            point->y,
            point->z;
    target_pt.z() = 0;
    final_goal = target_pt;

    Eigen::Vector3i goal_idx = plannerManager->m_globalMap->coord2gridIndex(final_goal);
    if (plannerManager->m_globalMap->isOccupied(goal_idx, false)) {
        Eigen::Vector3i goal_neigh_idx;
        if (plannerManager->m_astar_path_finder->findNeighPoint(goal_idx, goal_neigh_idx, 2)) {
            goal_idx = goal_neigh_idx;
        }
    }
    final_goal = plannerManager->m_globalMap->gridIndex2coord(goal_idx);
    if (plannerManager->pathFinding(start_pt, final_goal, robot_cur_speed)){
        have_target = true;
        sentryStatus = planningStatus::GEN_NEW_TRAJ;
    }
}


/**
 * @brief       抵达目标位置服务
 * @param
 */
bool ReplanFSM::srvGoTargetCallBack(sentry_msgs::GoTarget::Request &target,
                                 sentry_msgs::GoTarget::Response &res)
{
    Eigen::Vector3d target_pt;
    Eigen::Vector3d last_target_pt;
    int motion_mode;

    target_pt(0) = target.target_x;
    target_pt(1) = target.target_y;
    motion_mode = target.target_mode;

    ROS_WARN("[node] receive the decision target, (x, y, mode): (%.2f, %.2f) motion planning mode: %d", target.target_x, target.target_y, motion_mode);
    decision_mode = motion_mode; /// 决策模式

    if(distance(start_pt, target_pt) < 0.1){
        res.rst = true;
        return true;
    }

    // 如果接收到的target与实际相差不太大的话，就认为是同一个点
    if(distance(target_pt, final_goal) > 0.3 || !(planning_succeed)){
        final_goal = target_pt;
        have_target = true;
    }else{
        res.rst = true;
        return true;
    }

    Eigen::Vector3i goal_idx = plannerManager->m_globalMap->coord2gridIndex(final_goal);
    if (plannerManager->m_globalMap->isOccupied(goal_idx, false)) {
        Eigen::Vector3i goal_neigh_idx;
        if (plannerManager->m_astar_path_finder->findNeighPoint(goal_idx, goal_neigh_idx, 2)) {
            goal_idx = goal_neigh_idx;
        }
    }
    final_goal = plannerManager->m_globalMap->gridIndex2coord(goal_idx);
    final_goal.z() = plannerManager->m_globalMap->getHeight(goal_idx.x(), goal_idx.y());


    if (plannerManager->pathFinding(start_pt, final_goal, robot_cur_speed)){
        /// 追击模式下出现超级绕弯的点扔掉！！
        ROS_DEBUG("[FSM] path_length: %f", plannerManager->m_referenceSmooth->traj_length);
        if(motion_mode == 5 && plannerManager->m_referenceSmooth->traj_length > 12.0 &&
            distance(start_pt, final_goal) * 3 < plannerManager->m_referenceSmooth->traj_length){
            planning_succeed = false;
            ROS_ERROR("return succeed false");
            res.rst = false;
            have_target = false;
            final_goal = start_pt;
        }else{
            sentryStatus = planningStatus::GEN_NEW_TRAJ;
            planning_succeed = true;
            res.rst = true;
            have_target = true;
        }
    }else{
        planning_succeed = false;
        ROS_ERROR("return succeed false");
        res.rst = false;
        have_target = false;
    }
    return true;
}