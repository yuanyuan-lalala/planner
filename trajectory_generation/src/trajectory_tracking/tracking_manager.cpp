#include "tracking_manager.h"
#include "RM_GridMap.h"
#include <numeric>
//#include "../include/visualization_utils.h"



void tracking_manager::init(ros::NodeHandle &nh)
{
    nh.param("tracking_node/wheel_tread", robot_wheel_tread, 0.42);
    nh.param("tracking_node/target_point_arrive_radius", target_point_arrive_radius, 0.05);
    nh.param("tracking_node/occ_file_path", occ_file_path, std::string("occfinal.png"));
    nh.param("tracking_node/bev_file_path", bev_file_path, std::string("bevfinal.png"));
    nh.param("tracking_node/distance_map_file_path", distance_map_file_path, std::string("distance.png"));

    nh.param("tracking_node/map_resolution", map_resolution, 0.1);
    nh.param("tracking_node/map_lower_point_x", map_lower_point(0), 0.0);
    nh.param("tracking_node/map_lower_point_y", map_lower_point(1), 0.0);
    nh.param("tracking_node/map_lower_point_z", map_lower_point(2), 0.0);
    nh.param("tracking_node/map_x_size", map_x_size, 28.0);
    nh.param("tracking_node/map_y_size", map_y_size, 15.0);
    nh.param("tracking_node/map_z_size", map_z_size, 2.0);
    nh.param("tracking_node/search_height_min", search_height_min, 0.1);
    nh.param("tracking_node/search_height_max", search_height_max, 1.2);
    nh.param("tracking_node/search_radius", search_radius, 5.0);
    nh.param("tracking_node/robot_radius_dash", robot_radius_dash, 0.1);
    nh.param("tracking_node/robot_radius", robot_radius, 0.3);

    ROS_WARN("map/occ_file_path: %s", occ_file_path.c_str());
    ROS_WARN("map/bev_file_path: %s", bev_file_path.c_str());
    ROS_WARN("map/distance_map_file_path: %s", distance_map_file_path.c_str());


    sentry_wheelyawspeed_feedback = nh.subscribe("/slaver/wheel_state", 2, &tracking_manager::rcvSentryWheelSpeedYawCallback, this);
    gazebo_real_pos_sub = nh.subscribe("/gazebo/model_states", 1, &tracking_manager::rcvGazeboRealPosCallback, this, ros::TransportHints().unreliable().reliable().tcpNoDelay());
    lidar_imu_pos_sub = nh.subscribe("/odometry_imu", 1, &tracking_manager::rcvLidarIMUPosCallback, this, ros::TransportHints().unreliable().reliable().tcpNoDelay());
//    planning_mode_sub = nh.subscribe("/xtl_mode", 1, &tracking_manager::rcvPlanningModeCallback, this);

    robot_hp_sub = nh.subscribe("/slaver/robot_HP", 1, &tracking_manager::rcvRobotHPCallback, this);
    robot_status_sub =
            nh.subscribe("/slaver/robot_status", 1, &tracking_manager::rcvRobotStatusCallback, this);
    sentry_HP_sub = nh.subscribe("/slaver/robot_HP", 1, &tracking_manager::rcvSentryHPJudgeCallback, this);

    robot_left_speed_pub = nh.advertise<std_msgs::Float64>("/mbot/left_wheel_joint_controller/command", 1);
    robot_right_speed_pub = nh.advertise<std_msgs::Float64>("/mbot/right_wheel_joint_controller/command", 1);
    sentry_speed_pub = nh.advertise<planner::slaver_speed>("/sentry_des_speed", 1);
    m_imu_acc_pub = nh.advertise<std_msgs::Float64>("/imu_filter", 1);
    m_kal_acc_pub = nh.advertise<std_msgs::Float64>("/kal_filter", 1);
    m_solver_status_pub = nh.advertise<std_msgs::Bool>("/solver_status", 1);
    global_replan_pub = nh.advertise<std_msgs::Bool>("/replan_flag", 1);
    robot_cur_yaw_pub = nh.advertise<std_msgs::Float64>("/robot_cur_yaw_reg", 1);

    map_inv_resolution = 1.0 / map_resolution;
    map_upper_point(0) = map_lower_point(0) + map_x_size;
    map_upper_point(1) = map_lower_point(1) + map_y_size;
    map_upper_point(2) = map_lower_point(2) + map_z_size;

    grid_max_id_x = (int)(map_x_size * map_inv_resolution);
    grid_max_id_y = (int)(map_y_size * map_inv_resolution);
    grid_max_id_z = (int)(map_z_size * map_inv_resolution);

    global_map.reset(new GlobalMap_track);
    cv::Mat occ_map3c;
    occ_map3c = cv::imread(occ_file_path);
    std::vector <cv::Mat> channels;
    cv::split(occ_map3c, channels);
    cv::Mat occ_map = channels.at(0);

    global_map->initGridMap(nh, occ_map, bev_file_path, distance_map_file_path, map_resolution,
                            map_lower_point, map_upper_point, grid_max_id_x, grid_max_id_y, grid_max_id_z,
                            robot_radius, search_height_min, search_height_max, search_radius); ///////////////

    localplanner.reset(new LocalPlanner);
    localplanner->init(nh, global_map);

    vislization_util.reset(new Vislization);
    vislization_util->init(nh);

    planningMode = planningType::FASTMOTION;  // 初始为快速移动模式
}

void tracking_manager::gazeboVelAccSmooth(double speed, double dt)  // 对gazebo的速度加速度进行kalman平滑处理
{
    Eigen::Vector2d measure_speed;
    measure_speed << speed, 0;

    if(!gazebo_accvel_filter.m_filter_inited)
    {
        gazebo_accvel_filter.initParam(1e-4, 1e-4, 0.004, false);
        gazebo_accvel_filter.m_filter_inited = true;
        gazebo_accvel_filter.setState(measure_speed);
    }
    else
    {
        gazebo_accvel_filter.predictUpdate(dt);
        gazebo_accvel_filter.measureUpdate(measure_speed);
    }
}

void tracking_manager::velocityYawSmooth(double velocity_yaw, double dt)  // 对速度和yaw进行平滑处理
{
    Eigen::Vector2d measure_speedyaw;
    Eigen::Vector2d speed_yaw_temp;
    measure_speedyaw << velocity_yaw, 0;
    kalman_filter_speed_yaw.getResults(speed_yaw_temp);
    if(measure_speedyaw(0) - speed_yaw_temp(0) > M_PI){
        kalman_filter_speed_yaw.setState(measure_speedyaw);
    }

    if(!kalman_filter_speed_yaw.m_filter_inited)
    {
        kalman_filter_speed_yaw.initParam(tracking_manager::speed_init_r, tracking_manager::speed_init_q, 0.01, false);
        kalman_filter_speed_yaw.m_filter_inited = true;
        kalman_filter_speed_yaw.setState(measure_speedyaw);
    }
    else
    {
        kalman_filter_speed_yaw.predictUpdate(dt, tracking_manager::robot_odometry_speed);
        kalman_filter_speed_yaw.measureUpdate(measure_speedyaw);
    }
}

void tracking_manager::lidarSmooth(Eigen::Vector3d position, Eigen::Vector3d velocity, double dt)
{  // 雷达定位相关反馈数据滤波平滑
    Eigen::Vector2d measurevalue_x;
    Eigen::Vector2d measurevalue_y;
    Eigen::Vector2d measurevalue_yaw;

    measurevalue_x << position(0), 0;
    measurevalue_y << position(1), 0;
    measurevalue_yaw << position(2), velocity(2);
    Eigen::Vector2d yaw_temp;
    kalman_filter_yaw.getResults(yaw_temp);

    if(abs(measurevalue_yaw(0) - yaw_temp(0)) > M_PI )  /// 防止角度跳变
    {
        kalman_filter_yaw.setState(measurevalue_yaw);
    }

    if (!kalman_filter_speedx.m_filter_inited)
    {
        kalman_filter_speedx.initParam(tracking_manager::speed_init_r, tracking_manager::speed_init_q, 0.01, false);
        kalman_filter_speedy.initParam(tracking_manager::speed_init_r, tracking_manager::speed_init_q, 0.01, false);
        kalman_filter_yaw.initParam(tracking_manager::yaw_init_r, tracking_manager::yaw_init_q, 0.01, false);

        ROS_ERROR("start init");
        kalman_filter_speedx.m_filter_inited = true;
        kalman_filter_speedx.setState(measurevalue_x);

        kalman_filter_speedy.m_filter_inited = true;
        kalman_filter_speedy.setState(measurevalue_y);

        kalman_filter_yaw.m_filter_inited = true;
        kalman_filter_yaw.setState(measurevalue_yaw);
    }
    else
    {
        kalman_filter_speedx.predictUpdate(dt);
        kalman_filter_speedx.measureUpdate(measurevalue_x);

        kalman_filter_speedy.predictUpdate(dt);
        kalman_filter_speedy.measureUpdate(measurevalue_y);

        kalman_filter_yaw.predictUpdate(dt);
        kalman_filter_yaw.measureUpdate(measurevalue_yaw);
    }
}

void tracking_manager::rcvDebugFlag(const std_msgs::BoolConstPtr &msg) {
    tracking_manager::visualization_flag = msg->data;
    ROS_WARN("Visualization Flag Change");
}

void tracking_manager::rcvRobotHPCallback(const sentry_msgs::RobotsHPConstPtr &msg) {
    if (sentryColor == teamColor::red) {
        mate_outpost_hp = msg->red_outpost_hp;
    } else {
        mate_outpost_hp = msg->blue_outpost_hp;
    }
    ROS_WARN("mate_outpost_hp: %f", mate_outpost_hp);

    if (mate_outpost_hp < 1) {
        if(!isxtl){
            replan_now = true;
        }
        isxtl = true;  //  前哨站血量够低的话进小陀螺
    }else{
        isxtl = false;
    }
}

void tracking_manager::rcvSentryHPJudgeCallback(const sentry_msgs::RobotsHPConstPtr &msg)
{
    countHP ++;   /// 血量返回计数
    int current_HP;
    if(sentryColor == teamColor::red){  // 红蓝方判断
        current_HP = msg->red_sentry_hp;
    }else{
        current_HP = msg->blue_sentry_hp;
    }
    ROS_WARN("[FSM SentryHP] current_HP: %f", current_HP);

    if(sentry_HP == 0){  /// 初始化当前的烧饼血量
        sentry_HP = current_HP;
    }

    if((sentry_HP - current_HP) >= 10){ /// 血量减少超过10，说明被攻击，开始计数并转换标志位
        countHP = 0;
        is_attacked = true;
        ROS_ERROR("[FSM SentryHP] is_attacked!!");
    }
    else if(countHP > 20 && (sentry_HP == current_HP))
    {
        is_attacked = false;
        ROS_WARN("[FSM SentryHP] Safe!!");
    }
    sentry_HP = current_HP;
    return;
}

void tracking_manager::rcvRobotStatusCallback(const sentry_msgs::RobotStatusConstPtr &msg) {

    if (msg->id == 7) {  //! 蓝色107
        sentryColor = teamColor::red;
    } else {
        sentryColor = teamColor::blue;
    }
//    ROS_WARN("sentryColor: %d", (int)sentryColor);
}


void tracking_manager::rcvSentryWheelSpeedYawCallback(const geometry_msgs::Vector3ConstPtr &state)
{
    if(abs(state->x) < 10.0){
        tracking_manager::robot_wheel_speed = state->x;
    }
    if(abs(tracking_manager::robot_wheel_yaw - state->y* M_PI / 180) < 10000){
        tracking_manager::robot_wheel_yaw = state->y * M_PI / 180;
    }
}

void tracking_manager::rcvPlanningModeCallback(const std_msgs::Int64ConstPtr &msg)
{
    isxtl = msg->data < 3 ? true: false;
//    ROS_WARN("planningMode = %d", isxtl);  // 查看模式
}

void tracking_manager::rcvLidarIMUPosCallback(const nav_msgs::OdometryConstPtr &state)
{
    /* 实时位置 */
    control_time = ros::Time::now(); // 当此控制周期的时间
    double cost_time = (control_time - localplanner->start_tracking_time).toSec();

    /* 实时位置 */
    geometry_msgs::Pose pose;
    geometry_msgs::Twist twist;
    pose = state->pose.pose;
    twist = state->twist.twist;
    robot_cur_position(0) = pose.position.x;
    robot_cur_position(1) = pose.position.y;
    robot_cur_position(2) = 0.0;

    global_map->odom_position(0) = pose.position.x;
    global_map->odom_position(1) = pose.position.y;
    global_map->odom_position(2) = pose.position.z;

    double robot_x = pose.position.x;
    double robot_y = pose.position.y;

    double x = pose.orientation.x;
    double y = pose.orientation.y;
    double z = pose.orientation.z;
    double w = pose.orientation.w;
    double siny_cosp = +2.0 * (w * z + x * y);
    double cosy_cosp = +1.0 - 2.0 * (y * y + z * z);
    double robot_add_yaw = 0.0;  // TODO 这里可能会有点问题，在非陀螺模式下不保证底盘一定在+-180度内，如果出现异常情况的话会疯转
    double robot_lidar_yaw = atan2f(siny_cosp, cosy_cosp);

    double robot_yaw_temp = robot_wheel_yaw;
    int k = robot_yaw_temp / (2 * M_PI);
    robot_yaw_temp = robot_yaw_temp - k * (2 * M_PI);

    int j = robot_yaw_temp / M_PI;
    robot_yaw_temp = robot_yaw_temp - j * (2 * M_PI);
    robot_cur_yaw = robot_yaw_temp;
    robot_add_yaw = (k+j)*(2*M_PI);

    double v_ctrl = 0.0;
    double phi_ctrl = 0.0;
    double acc_ctrl = 0.0;
    double angular_ctrl = 0.0;
    Eigen::Vector4d predict_input;

    robot_cur_speed(0) = robot_wheel_speed * cos(robot_cur_yaw);
    robot_cur_speed(1) = robot_wheel_speed * sin(robot_cur_yaw);

    //  判断一下是不是需要加入抵达终点的判断来set速度为0
    double line_speed = sqrt(pow(abs(robot_cur_speed(0)), 2) + pow(abs(robot_cur_speed(1)), 2));
    double line_speed_temp = sqrt(pow(abs(twist.linear.x), 2) + pow(abs(twist.linear.y), 2));

    Eigen::VectorXd sentry_state;
    std::vector<Eigen::Vector4d> state_deque;
    std::vector<Eigen::Vector2d> input_deque;

    sentry_state = Eigen::VectorXd::Zero(4);
    sentry_state << robot_x, robot_y, line_speed, robot_cur_yaw;
//    localplanner->ref_phi.clear();
//    localplanner->ref_speed.clear();
//    localplanner->ref_trajectory.clear();
//    localplanner->ref_phi.push_back(robot_cur_yaw);
    if(localplanner->m_get_global_trajectory){
        arrival_goal = false;
    }
    if(arrival_goal){
        v_ctrl = 0.0;
        phi_ctrl = robot_lidar_yaw;
        acc_ctrl = 0.0;
        angular_ctrl = 0.0;
        Eigen::Vector2d optimal_differential_speed;
        Eigen::Vector4d MPC_Control;

        optimal_differential_speed = getDifferentialModelSpeed(v_ctrl, angular_ctrl, robot_wheel_tread);
        publishMbotOptimalSpeed(optimal_differential_speed);

        MPC_Control(0) = v_ctrl;
        MPC_Control(1) = phi_ctrl;
        MPC_Control(2) = acc_ctrl;
        MPC_Control(3) = checkMotionMode();
        in_bridge = false;
        publishSentryOptimalSpeed(MPC_Control);

        return;
    }

    double target_distance = (localplanner->target_point - robot_cur_position).norm();
    if(target_distance < 0.3 && (localplanner->motion_mode != 8)){
        ROS_WARN("[MPC] arrival_goal!");
        arrival_goal = true;
    }

    if(planningMode == 3){
        localplanner->getFightTrackingTraj(robot_cur_position, cost_time, robot_cur_yaw);  // 正常运动快速移动
    }else{
        localplanner->getFightTrackingTraj(robot_cur_position, cost_time, robot_cur_yaw);  // 可以倒车模式，更加灵活,团战模式全部适合倒车
    }

    if(localplanner->reference_path.size() > 2){
        checkMotionNormal(line_speed_temp);
        int ret = localplanner->solveNMPC(sentry_state);
        localplanner->getNMPCPredictXU(predict_input); // v phi a w
        localplanner->getNMPCPredictionDeque(state_deque, input_deque);


        bool collision = localplanner->checkfeasible();
        if(collision){
            ROS_WARN("MPC is not safe");
        }
        vislization_util->visCandidateTrajectory(state_deque);
        vislization_util->visReferenceTrajectory(localplanner->ref_trajectory);
        vislization_util->visObsCenterPoints(localplanner->obs_points);


        replan_check_flag.insert(replan_check_flag.begin(), collision);
        if(replan_check_flag.size() > 25){  // 检测num帧的数据防止碰撞，实际检测秒数为num/100帧
            replan_check_flag.pop_back();
        }
    }
    for(int i = 0; i < state_deque.size() / 2; i++){
        std::cout<<state_deque[i].z()<<std::endl;
    }
    checkReplanFlag();


    v_ctrl = predict_input(0);
    phi_ctrl = predict_input(1);
    acc_ctrl = predict_input(2);
    angular_ctrl = predict_input(3);

    if(planningMode == 4){  // 摆脱模式处理
        double reverse_time = (ros::Time::now() - dispense_time).toSec();
        if(reverse_time > 0.5){
            replan_now = true;
        }
        v_ctrl = -0.5 * v_ctrl;  // 卡住你就倒车拐弯
    }


    Eigen::Vector4d MPC_Control;
    MPC_Control(0) = phi_ctrl + robot_add_yaw;
    MPC_Control(1) = robot_lidar_yaw;
    MPC_Control(2) = v_ctrl;
    MPC_Control(3) = checkMotionMode();
//    if(isxtl && localplanner->checkXtl()){
//        MPC_Control(3) = 1;
//    }else{
//        MPC_Control(3) = 0;
//    }
    in_bridge = localplanner->checkBridge();
    publishSentryOptimalSpeed(MPC_Control);
    std::cout<<"[END] end local planning"<<std::endl;
}

void tracking_manager::rcvGazeboRealPosCallback(const gazebo_msgs::ModelStatesConstPtr &state)
{
    /* 实时位置 */
    int robot_namespace_id = 0;
    for(int i = 0;i < state->name.size();i++){
        if(state->name[i] == "mbot"){
            robot_namespace_id = i;
            break;
        }
    }

    ros::Time end_time = ros::Time::now();
    double time = (end_time - control_time).toSec();
    control_time = ros::Time::now(); // 当此控制周期的时间
    double cost_time = (control_time - localplanner->start_tracking_time).toSec();

    robot_cur_position(0) = state->pose[robot_namespace_id].position.x;
    robot_cur_position(1) = state->pose[robot_namespace_id].position.y;
    robot_cur_position(2) = 0.0;

    global_map->odom_position(0) = state->pose[robot_namespace_id].position.x;
    global_map->odom_position(1) = state->pose[robot_namespace_id].position.y;
    global_map->odom_position(2) = state->pose[robot_namespace_id].position.z;

    double robot_x = state->pose[robot_namespace_id].position.x;
    double robot_y = state->pose[robot_namespace_id].position.y;

    double x = state->pose[robot_namespace_id].orientation.x;
    double y = state->pose[robot_namespace_id].orientation.y;
    double z = state->pose[robot_namespace_id].orientation.z;
    double w = state->pose[robot_namespace_id].orientation.w;
    double siny_cosp = +2.0 * (w * z + x * y);
    double cosy_cosp = +1.0 - 2.0 * (y * y + z * z);
    robot_cur_yaw = atan2f(siny_cosp, cosy_cosp);

    // yaw pitch roll
    robot_cur_posture(2) = atan2f(siny_cosp, cosy_cosp);
    robot_cur_posture(1) = asin(2 * (w * y - x * z));
    robot_cur_posture(0) = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));

    global_map->odom_posture = robot_cur_posture;

    double v_ctrl = 0.0;
    double phi_ctrl = 0.0;
    double acc_ctrl = 0.0;
    double angular_ctrl = 0.0;
    Eigen::Vector4d predict_input;

    //  判断一下是不是需要加入抵达终点的判断来set速度为0

    double line_speed = sqrt(pow(abs(state->twist[robot_namespace_id].linear.x), 2) + pow(abs(state->twist[robot_namespace_id].linear.y), 2));
    Eigen::Vector2d robot_accvel;

    robot_cur_speed(0) = state->twist[robot_namespace_id].linear.x;
    robot_cur_speed(1) = state->twist[robot_namespace_id].linear.y;

    Eigen::VectorXd sentry_state;
    std::vector<Eigen::Vector4d> state_deque;
    std::vector<Eigen::Vector2d> input_deque;

    sentry_state = Eigen::VectorXd::Zero(4);
    sentry_state << robot_x, robot_y, line_speed, robot_cur_yaw;


    localplanner->robot_cur_pitch = asin(-2 * (x * z - w * y));

    if(localplanner->m_get_global_trajectory){
        arrival_goal = false;
    }
    if(arrival_goal){
        v_ctrl = 0.0;
        phi_ctrl = robot_cur_yaw;
        acc_ctrl = 0.0;
        angular_ctrl = 0.0;
        Eigen::Vector2d optimal_differential_speed;
        Eigen::Vector4d MPC_Control;

        optimal_differential_speed = getDifferentialModelSpeed(v_ctrl, angular_ctrl, robot_wheel_tread);
        publishMbotOptimalSpeed(optimal_differential_speed);

        MPC_Control(0) = v_ctrl;
        MPC_Control(1) = phi_ctrl;
        MPC_Control(2) = angular_ctrl;
        MPC_Control(3) = checkMotionMode();
//        MPC_Control(3) = 1;
        in_bridge = false;
        publishSentryOptimalSpeed(MPC_Control);
        return;
    }

    double target_distance = (localplanner->target_point - robot_cur_position).norm();
    if(target_distance < 0.1){
        ROS_WARN("[MPC] arrival_goal!");
        arrival_goal = true;
    }
    if(planningMode == 3){
//        localplanner->getTrackingTraj(robot_cur_position, cost_time, robot_cur_yaw);  // 正常运动快速移动
        localplanner->getFightTrackingTraj(robot_cur_position, cost_time, robot_cur_yaw);
    }else{
        localplanner->getFightTrackingTraj(robot_cur_position, cost_time, robot_cur_yaw);  // 可以倒车模式，更加灵活
    }

    if(localplanner->reference_path.size() > 2){
        checkMotionNormal(line_speed);

        int ret = localplanner->solveNMPC(sentry_state);
        localplanner->getNMPCPredictXU(predict_input); // v phi a w
        localplanner->getNMPCPredictionDeque(state_deque, input_deque);
        bool collision = localplanner->checkfeasible();
        if(collision){
            ROS_WARN("MPC is not safe");
        }
        vislization_util->visCandidateTrajectory(state_deque);
        vislization_util->visReferenceTrajectory(localplanner->ref_trajectory);
        vislization_util->visObsCenterPoints(localplanner->obs_points);

        replan_check_flag.insert(replan_check_flag.begin(), collision);
        if(replan_check_flag.size() > 250){ // 检测num帧的数据防止碰撞，实际检测秒数为num/800帧
            replan_check_flag.pop_back();
        }
    }
    checkReplanFlag();

    v_ctrl = predict_input(0);
    phi_ctrl = predict_input(1);
    acc_ctrl = predict_input(2);
    angular_ctrl = predict_input(3);

    /* 获取四轮差速模型的左右轮轮速 */
    Eigen::Vector2d optimal_differential_speed;
    Eigen::Vector4d MPC_Control;

    if(planningMode == 4){
        double reverse_time = (ros::Time::now() - dispense_time).toSec();
        if(reverse_time > 0.6){
            replan_now = true;
        }
        v_ctrl = -0.5 * v_ctrl;  // 卡住你就倒车拐弯
    }

    optimal_differential_speed = getDifferentialModelSpeed(v_ctrl, angular_ctrl, robot_wheel_tread);
    publishMbotOptimalSpeed(optimal_differential_speed);

    MPC_Control(0) = v_ctrl;
    MPC_Control(1) = phi_ctrl;
    MPC_Control(2) = angular_ctrl;
    MPC_Control(3) = checkMotionMode();
//    if(isxtl && localplanner->checkXtl()){
//        MPC_Control(3) = checkMotionMode();
//    }else{
//        MPC_Control(3) = 0;
//    }
    in_bridge = localplanner->checkBridge();
    publishSentryOptimalSpeed(MPC_Control);
}

/**
 * @brief 用线速度和角速度映射到左右轮轮速
 * @param
 */
Eigen::Vector2d tracking_manager::getDifferentialModelSpeed(double line_speed, double angular, double wheel_tread)
{
    Eigen::Vector2d LR_speed;
    LR_speed(0) = line_speed + wheel_tread / 2 * angular;
    LR_speed(1) = line_speed - wheel_tread / 2 * angular;
    return LR_speed;
}



int tracking_manager::checkMotionMode(){
    /**
     * 判断底盘运动模式,分别为正常运行，可倒车模式，慢速陀螺/小虎步，快速陀螺
     */
    int xtl_mode = 0;
    double time = accumulate(localplanner->duration_time.begin(), localplanner->duration_time.end(), 0.0);
    if(!isxtl){  // 正常运行，可倒车模式
        if(arrival_goal){
            xtl_mode = 2;
        }
        else if(time > 0.5 && !localplanner->checkBridge() && planningMode != planningType::DISPENSE){
           //  time > 2.0 &&
           xtl_mode = 0;  // 轨迹够长且不在桥洞里，在桥洞里不准转弯
        }else{
            xtl_mode = 1;
        }
    }else if(!arrival_goal && isxtl && localplanner->checkXtl()){
        if(is_attacked || localplanner->motion_mode == 8 || localplanner->motion_mode == 11){
            xtl_mode = 2;
        }else{
            xtl_mode = 0;
        }
//        xtl_mode = 2;
    }else if(arrival_goal && isxtl && localplanner->checkXtl()){
        if(is_attacked || localplanner->motion_mode == 11){
            xtl_mode = 3;
        }else{
            xtl_mode = 2;
        }
//        xtl_mode = 2;
    }else{
        if(localplanner->checkBridge()){
            xtl_mode = 1;
        }else{
            xtl_mode = 0;
        }
    }
    return xtl_mode;
}

void tracking_manager::checkReplanFlag(){
    if(localplanner->m_get_global_trajectory){
        replan_check_flag.clear();
        localplanner->m_get_global_trajectory = false;
    }
    int num_collision_error = accumulate(replan_check_flag.begin(), replan_check_flag.end(), 0);
    int num_tracking_low = accumulate(localplanner->tracking_low_check_flag.begin(), localplanner->tracking_low_check_flag.end(), 0);
    std_msgs::Bool replan_flag;
//    std::cout<<"num_collision_error: "<<num_collision_error<<std::endl;
    if(num_collision_error > 200 || num_tracking_low > 6 || replan_now){  // 实车参数为15，这里200为仿真参数
        ROS_ERROR("need to replan now !!");
        planningMode = planningType::FASTMOTION;
        replan_flag.data = true;
    }
    else{
        replan_flag.data = false;
    }
    global_replan_pub.publish(replan_flag);
}

void tracking_manager::checkMotionNormal(double line_speed)
{
    /**
     * @brief 检查tracking是否正常，是否被卡住需要摆脱
     */
     if(localplanner->m_get_global_trajectory){
         planningMode = planningType::FASTMOTION;
         replan_now = false;
         start_checking_time = ros::Time::now();
         speed_check.clear();
     }
    speed_check.insert(speed_check.begin(), line_speed);
    double tracking_time = (ros::Time::now() - start_checking_time).toSec();
    double max_speed = 1.0;
    if(tracking_time > 1.2){  // 限制两秒钟的速度序列
        speed_check.pop_back();
        max_speed = *max_element(speed_check.begin(), speed_check.end());
        double average_speed = std::accumulate(speed_check.begin(), speed_check.end(), 0.0) / speed_check.size();
        if(max_speed < 0.8 && !arrival_goal && average_speed < 0.3){
//            ROS_ERROR("dispense form now!");
            if(planningMode != 4){
                ROS_ERROR("get dispense time");
                dispense_time = ros::Time::now();  // 进入摆脱模式时进行计时
            }
            planningMode = planningType::DISPENSE;
        }

    }
}


void tracking_manager::publishMbotOptimalSpeed(Eigen::Vector2d &speed)
{
    std_msgs::Float64 left_speed;
    std_msgs::Float64 right_speed;
    planner::slaver_speed sentry_speed;

    /* 抵达规划路径终点时立即停止移动 */
    left_speed.data = speed(0) / 0.1;
    right_speed.data = speed(1) / 0.1;

    tracking_manager::robot_left_speed_pub.publish(left_speed);
    tracking_manager::robot_right_speed_pub.publish(right_speed);
}

void tracking_manager::publishSentryOptimalSpeed(Eigen::Vector4d &speed)
{   // TODO 改为x y v w theta
    // v phi a w
    planner::slaver_speed sentry_speed;
    std::pair<int, int> count_mode = {0,0};  /// max_id max_count

    motion_mode_vec.insert(motion_mode_vec.begin(), int(speed(3)));
    if(motion_mode_vec.size() > 40){
        motion_mode_vec.pop_back();
    }
    for(int i = 0; i < 4; i++){
        if(std::count(motion_mode_vec.begin(), motion_mode_vec.end(), i) > count_mode.second){
            count_mode.first = i;
            count_mode.second = std::count(motion_mode_vec.begin(), motion_mode_vec.end(), i);
        }
    }

//    std::cout<<"mpc_control(3): "<<speed(3)<<"count_mode.first: "<<count_mode.first<<std::endl;


    /* 抵达规划路径终点时立即停止移动 */

    sentry_speed.angle_target = speed(0);  // 目标yaw角度
    sentry_speed.angle_current = speed(1);  // 当前的雷达yaw角度
//    sentry_speed.angle_current = speed(1) + M_PI;  // 当前的雷达yaw角度
    sentry_speed.line_speed = speed(2);  // 目标线速度
    if(count_mode.second > 38){
        sentry_speed.xtl_flag = count_mode.first;  // 目标模式
    }else{
        sentry_speed.xtl_flag = last_motion_xtl_flag;  // 目标模式
    }
//    sentry_speed.xtl_flag = count_mode.first;  // 目标模式
    last_motion_xtl_flag = sentry_speed.xtl_flag;

//    sentry_speed.xtl_flag = 0;  // 目标模式
    sentry_speed.in_bridge = in_bridge;   // 是否进入桥洞

    std_msgs::Float64 robot_yaw;
    robot_yaw.data = tracking_manager::robot_cur_yaw;

    tracking_manager::sentry_speed_pub.publish(sentry_speed);
    robot_cur_yaw_pub.publish(robot_yaw);
}

