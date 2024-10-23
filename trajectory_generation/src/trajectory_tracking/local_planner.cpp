//
// Created by hitcrt on 2023/4/5.
//

#include "local_planner.h"
#include <math.h>
#include <numeric>
#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>

const int LocalPlanner::n;
const int LocalPlanner::m;

void LocalPlanner::linearation(double time, double robot_cur_yaw)
{
    // MPC参考轨迹，速度，时间，预测轨迹和输入全部清空
    ref_phi.clear();
    ref_speed.clear();
    ref_phi.push_back(robot_cur_yaw);

    observation.time = time;
    ref_trajectory.clear();
    ref_velocity.clear();
    reference_time.clear();
    predictState.clear();
    predictInput.clear();
    obs_points.clear();
    obs_points_t.clear();
    obs_num = 0;
    double sum_time = accumulate(ref_time.begin(), ref_time.end(), 0.0);

    int segment_index;
    double total_time;  // 该段前的总时间
    getTrackingSegmentIndex(time, segment_index, total_time);
    double segment_time = time - total_time;
    bool unknown_obs_exist = false;
    std::vector<bool> occ_flag;  // 记录参考点中哪个点在障碍物中

    bool evade_flag = false;
    if(motion_mode == 8){  // 防守进攻模式下
        if((global_map->odom_position.head(2) - target_point.head(2)).norm() < 1.2){
            evade_flag = true;
        }else{
            evade_flag = false;
        }
    }

    if(!evade_flag) {
        for (int i = 0; i < 20; i++) {  /// 获取参考轨迹与障碍物处理，如果在障碍物里直接进行处理
            if (segment_index < reference_velocity.size() - 1) //根据当前的时间判断是不是还在全局参考的index内
            {
                Eigen::Vector3d velocity_temp;
                Eigen::Vector3d position_temp;

                velocity_temp.x() = reference_velocity[segment_index].x() +
                                    (reference_velocity[segment_index + 1].x() -
                                     reference_velocity[segment_index].x()) * segment_time / dt;
                velocity_temp.y() = reference_velocity[segment_index].y() +
                                    (reference_velocity[segment_index + 1].y() -
                                     reference_velocity[segment_index].y()) * segment_time / dt;
                velocity_temp.z() = 0.0;

                position_temp.x() = reference_path[segment_index].x() +
                                    (reference_path[segment_index + 1].x() - reference_path[segment_index].x()) *
                                    segment_time / dt;
                position_temp.y() = reference_path[segment_index].y() +
                                    (reference_path[segment_index + 1].y() - reference_path[segment_index].y()) *
                                    segment_time / dt;
                position_temp.z() = 0.0;

                ref_trajectory.push_back(position_temp);
                ref_velocity.push_back(velocity_temp);
                reference_time.push_back(time + i * dt);
                segment_index++;
            } else {  // 不在当前的index内，直接取最后一个点，直接获取规划的输入MPC参考轨迹速度时间
                ref_trajectory.push_back(reference_path[reference_path.size() - 1]);
                Eigen::Vector3d velocity_temp;
                velocity_temp = Eigen::Vector3d::Zero(3);
//            ref_velocity.push_back(velocity_temp);
                ref_velocity.push_back(reference_velocity[reference_velocity.size() - 1]);
                reference_time.push_back(time + i * dt);
            }

            Eigen::Vector3i pos_idx = global_map->coord2gridIndex(ref_trajectory[i]);
//        std::vector<Eigen::Vector3d> obs_point_temp;
//        std::vector<std::pair<int, Eigen::Vector3d>> obs_point_temp_t;
            if (!global_map->isOccupied(pos_idx.x(), pos_idx.y(), pos_idx.z())) {
                occ_flag.push_back(false);
//            obs_points_t.push_back(obs_point_temp_t);
            } else {
                occ_flag.push_back(true);
            }
        }
    }else{
        getCircleEvadeTraj(global_map->odom_position, ref_trajectory, ref_velocity, reference_time);
        for (int i = 0; i < 20; i++) {
            Eigen::Vector3i pos_idx = global_map->coord2gridIndex(ref_trajectory[i]);
            if (!global_map->isOccupied(pos_idx.x(), pos_idx.y(), pos_idx.z())) {
                occ_flag.push_back(false);
                std_msgs::Bool decision_flag;
                decision_flag.data = true;
                redecision_pub.publish(decision_flag);
            } else {
                std_msgs::Bool decision_flag;
                decision_flag.data = true;
                redecision_pub.publish(decision_flag);
                occ_flag.push_back(true);
            }
        }

        start_tracking_time = ros::Time::now();
    }

    for(int i = 0; i < 20; i++){
        /// 单独进行障碍物处理
        Eigen::Vector3i pos_idx = global_map->coord2gridIndex(ref_trajectory[i]);
        std::vector<Eigen::Vector3d> obs_point_temp;
        std::vector<std::pair<int, Eigen::Vector3d>> obs_point_temp_t;
//        std::cout<<occ_flag[i]<<std::endl;

        bool safe_constraint = occ_flag[i];
        // 前后七个参考点是否在障碍物内，但凡有一个在他就要考虑障碍物的问题
        safe_constraint = occ_flag[i] || (i > 18 ? false: occ_flag[i+1]) ||
                (i > 17 ? false: occ_flag[i+2]) || (i > 16? false:occ_flag[i+3]) ||
                (i < 1? false: occ_flag[i-1]) || (i < 2? false: occ_flag[i-2]) || (i < 3? false: occ_flag[i-3]);


        if(safe_constraint){
            if(!global_map->GridNodeMap[pos_idx.x()][pos_idx.y()]->exist_second_height){
                // 搜索最近的几个障碍物点(除了桥洞区域)
                for(int i = -10; i<=10; i++){
                    for(int j = -10; j<=10; j++){
                        Eigen::Vector3i temp_idx = {pos_idx.x() + i, pos_idx.y() + j, pos_idx.z()};
                        if(global_map->isOccupied(pos_idx.x() + i, pos_idx.y() + j, pos_idx.z()))
                        {
                            Eigen::Vector3d temp_pos = global_map->gridIndex2coord(temp_idx);
                            obs_point_temp.push_back(temp_pos);
                            if(global_map->isLocalOccupied(pos_idx.x() + i, pos_idx.y() + j, pos_idx.z())){
                                obs_point_temp_t.push_back(std::make_pair(1, temp_pos));
                                unknown_obs_exist = true;
                            }else{
                                obs_point_temp_t.push_back(std::make_pair(1, temp_pos));
                                unknown_obs_exist = true;
                            }
                        }
                    }
                }
            }
            obs_points_t.push_back(obs_point_temp_t);
        }
        else{
            obs_points_t.push_back(obs_point_temp_t);
        }
        obs_points.push_back(obs_point_temp);
        obs_num += obs_point_temp.size();
    }

    if(!unknown_obs_exist){  // 周围没有未知的动态障碍物时不进行局部避障
        for(int i = 0; i<obs_points_t.size(); i++){
            obs_points_t[i].clear();
        }
    }
    mpcInterface_.obsConstraintPtr_->timeTrajectory_ = reference_time;  // 障碍物约束参考轨迹设置
//    mpcInterface_.obsConstraintPtr_->obs_points_t_ = obs_points;
    mpcInterface_.obsConstraintPtr_->obs_points_t_ = obs_points_t;

    ocs2::RelaxedBarrierPenalty::Config barriercollisionPenaltyConfig(0.15, 0.1);
    stateCollisionSoftConstraintPtr = std::make_unique<ocs2::StateSoftConstraint>(std::make_unique<SentryCollisionConstraint>(mpcInterface_.obsConstraintPtr_), std::make_unique<ocs2::RelaxedBarrierPenalty>(barriercollisionPenaltyConfig));
    bool trues = mpcInterface_.problem_.stateSoftConstraintPtr->erase("stateCollisionBounds");  // 加入障碍物躲避约束
    mpcInterface_.problem_.stateSoftConstraintPtr->add("stateCollisionBounds", std::move(stateCollisionSoftConstraintPtr));
    mpcSolverPtr_.reset(new ocs2::SqpMpc(mpcInterface_.mpcSettings(), mpcInterface_.sqpSettings(), mpcInterface_.getOptimalControlProblem(), mpcInterface_.getInitializer()));
}

void LocalPlanner::getFightTrackingTraj(Eigen::Vector3d state, double time, double robot_cur_yaw)
{
    double min_dis = 100000;
    int min_id = 0;
    if(reference_path.size() < 2){
        return;
    }
    /// 获得当前的MPC的参考轨迹
    linearation(time, robot_cur_yaw);
    double dis = (state - ref_trajectory[0]).norm();
//    std::cout<<"state: "<<state.x()<<" "<<state.y()<<" "<<state.z()<<" "<<std::endl;
//    std::cout<<"ref_trajectory: "<<ref_trajectory[0].x()<<" "<<ref_trajectory[0].y()<<" "<<ref_trajectory[0].z()<<" "<<std::endl;
//    bool occlusion = lineVisib(state, ref_trajectory[0]);  // 判断当前的位置与参考轨迹之前是不是有碰撞或者具体太远
    if(dis > 0.4){  // 如果符合上述条件则reset参考时间保证整体运动稳定性
        ROS_WARN("tracking time low!");
        for(int i = 0; i<reference_path.size(); i++)  /// 得到当前位置最近且对应的时间最近的参考路径id
        {
//            if(abs(ref_time[i] - time) > 1.0){
//                continue;
//            }
            double distance = std::sqrt(pow(state(0) - reference_path[i](0), 2) + pow(state(1) - reference_path[i](1), 2));
            if(distance<min_dis){
                min_dis = distance;
                min_id = i;
            }
        }
        double reset_time = (min_id) * dt;
        linearation(reset_time, robot_cur_yaw);
        start_tracking_time = start_tracking_time + ros::Duration((time - reset_time));
        tracking_low_check_flag.insert(tracking_low_check_flag.begin(), 1.0);  // 如果连续多次滞后判断为出现规划问题，直接进入重规划程序
    }
    else{
        tracking_low_check_flag.insert(tracking_low_check_flag.begin(), 0.0);
    }

    if(tracking_low_check_flag.size() > 10){
        tracking_low_check_flag.pop_back();
    }

    double phi;
    double last_phi;

    for(int i = 0; i<planning_horizon; i++)  /// 这里我们更改只进行参考角度的更改
    {
        phi = atan2(ref_velocity[i](1),  ref_velocity[i](0));
        if(i == 0)  // 初始
        {
            if (phi - ref_phi[i] > M_PI) {
                phi = phi - 2 * M_PI;
            }
            if (phi - ref_phi[i] < -M_PI) {  // phi这里才是参考轨迹的yaw角
                phi = phi + 2 * M_PI;
            }
                // 参考路径与速度
            if (ref_phi[i] - phi > M_PI_2) {  // 参考角度与实际状态反馈的角度差pi
                ref_phi[i] = (phi + M_PI);  /// 相差一个周期
                speed_direction = -1;
                ref_speed.push_back(speed_direction * sqrt(pow(ref_velocity[i](1), 2) +
                                         pow(ref_velocity[i](0), 2)));
            }else if (ref_phi[i] - phi < -M_PI_2) {
                ref_phi[i] = (phi - M_PI);
                speed_direction = -1;
                ref_speed.push_back(speed_direction * sqrt(pow(ref_velocity[i](1), 2) +
                                         pow(ref_velocity[i](0), 2)));
            }else {  /// 小角度转向直接set
                ref_phi[i] = (phi);
                speed_direction = 1;
                ref_speed.push_back(speed_direction * sqrt(pow(ref_velocity[i](1), 2) +
                                         pow(ref_velocity[i](0), 2)));
            }
        }else{
            if (phi - last_phi > M_PI) {  /// 两个参考轨迹点之间差pi
                phi -=  2 * M_PI;
            } else if (phi - last_phi < -M_PI) {
                phi +=  2 * M_PI;
            }

            if (ref_phi[i - 1] - phi > M_PI_2) {   /// 平滑处理
                ref_phi.push_back(phi + M_PI);
                ref_speed.push_back(speed_direction * sqrt(pow(ref_velocity[i](1), 2) +
                                         pow(ref_velocity[i](0), 2)));
            } else if (ref_phi[i - 1] - phi < -M_PI_2) {
                ref_phi.push_back(phi - M_PI);
                ref_speed.push_back(speed_direction * sqrt(pow(ref_velocity[i](1), 2) +
                                         pow(ref_velocity[i](0), 2)));
            } else{
                ref_phi.push_back(phi);
                ref_speed.push_back(speed_direction * sqrt(pow(ref_velocity[i](1), 2) +
                                         pow(ref_velocity[i](0), 2)));
            }
        }
        last_phi = phi;
    }


}

void LocalPlanner::getTrackingTraj(Eigen::Vector3d state, double time, double robot_cur_yaw)
{
    double min_dis = 100000;
    int min_id = 0;
    if(reference_path.size() < 2){
        return;
    }
    /// 获得当前的MPC的参考轨迹
    linearation(time, robot_cur_yaw);
//    ROS_WARN("time: %f", time);
    double dis = (state - ref_trajectory[0]).norm();

    if(dis > 0.4){  // 如果符合上述条件则reset参考时间保证整体运动稳定性
        ROS_WARN("tracking time low!");
        for(int i = 0; i<reference_path.size(); i++)  /// 得到当前位置最近的参考路径id
        {
//            if(abs(ref_time[i] - time) > 1.0){
//                continue;
//            }
            double distance = std::sqrt(pow(state(0) - reference_path[i](0), 2) + pow(state(1) - reference_path[i](1), 2));
            if(distance<min_dis){
                min_dis = distance;
                min_id = i;
            }
        }
        double reset_time = (min_id) * dt;
        linearation(reset_time, robot_cur_yaw);
        start_tracking_time = start_tracking_time + ros::Duration((time - reset_time));
        tracking_low_check_flag.insert(tracking_low_check_flag.begin(), 1.0);  // 如果连续多次滞后判断为出现规划问题，直接进入重规划程序
    }
    else{
        tracking_low_check_flag.insert(tracking_low_check_flag.begin(), 0.0);
    }

    if(tracking_low_check_flag.size() > 10){
        tracking_low_check_flag.pop_back();
    }

    double phi;
    double last_phi;

    for(int i = 0; i<planning_horizon; i++)  /// 这里我们更改只进行参考角度的更改
    {
        phi = atan2(ref_velocity[i](1), ref_velocity[i](0));
        if(i == 0)  // 初始
        {
            // 参考路径与速度
            if (ref_phi[i] - phi > M_PI) {  // 参考角度与实际状态反馈的角度差pi
                ref_phi[i] = (phi + 2*M_PI);  /// 相差一个周期
                ref_speed.push_back(sqrt(pow(ref_velocity[i](1), 2) +
                                         pow(ref_velocity[i](0), 2)));
            }else if (ref_phi[i] - phi < -M_PI) {
                ref_phi[i] = (phi - 2 * M_PI);
                ref_speed.push_back(sqrt(pow(ref_velocity[i](1), 2) +
                                         pow(ref_velocity[i](0), 2)));
            }else {  /// 小角度转向直接set
                ref_phi[i] = (phi);
                ref_speed.push_back(sqrt(pow(ref_velocity[i](1), 2) +
                                         pow(ref_velocity[i](0), 2)));
            }
        }else{
            if (phi - last_phi > M_PI) {  /// 两个参考轨迹点之间差pi
                phi -=  2 * M_PI;
            } else if (phi - last_phi < -M_PI) {
                phi +=  2 * M_PI;
            }

            if (ref_phi[i - 1] - phi > M_PI) {   /// 平滑处理
                ref_phi.push_back(phi + 2 * M_PI);
                ref_speed.push_back(sqrt(pow(ref_velocity[i](1), 2) +
                                         pow(ref_velocity[i](0), 2)));
            } else if (ref_phi[i - 1] - phi < -M_PI) {
                ref_phi.push_back(phi - 2 * M_PI);
                ref_speed.push_back(sqrt(pow(ref_velocity[i](1), 2) +
                                         pow(ref_velocity[i](0), 2)));
            } else{
                ref_phi.push_back(phi);
                ref_speed.push_back(sqrt(pow(ref_velocity[i](1), 2) +
                                         pow(ref_velocity[i](0), 2)));
            }
        }
        last_phi = phi;
    }

}

void LocalPlanner::rcvGlobalTrajectory(const planner::trajectoryPolyConstPtr& polytraj)
{
    ROS_INFO("[Local Planner] rcv global trajectory");
    reference_path.clear();
    reference_velocity.clear();
    duration_time.clear();
    ref_time.clear();
    tracking_low_check_flag.clear();

    speed_direction = 1;
    int piece_num = polytraj->duration.size();
    m_polyMatrix_x.resize(piece_num, 4);
    m_polyMatrix_y.resize(piece_num, 4);
    // 设置每段轨迹的参数

    for(int i = 0; i<piece_num; i++)
    {
        duration_time.push_back(polytraj->duration[i]);
        m_polyMatrix_x(i, 0) = polytraj->coef_x[4 * i + 0];
        m_polyMatrix_x(i, 1) = polytraj->coef_x[4 * i + 1];
        m_polyMatrix_x(i, 2) = polytraj->coef_x[4 * i + 2];
        m_polyMatrix_x(i, 3) = polytraj->coef_x[4 * i + 3];
        m_polyMatrix_y(i, 0) = polytraj->coef_y[4 * i + 0];
        m_polyMatrix_y(i, 1) = polytraj->coef_y[4 * i + 1];
        m_polyMatrix_y(i, 2) = polytraj->coef_y[4 * i + 2];
        m_polyMatrix_y(i, 3) = polytraj->coef_y[4 * i + 3];
    }
    getRefTrajectory();  /// 根据轨迹的参数得到采样轨迹和采样速度
    getRefVel();
    start_tracking_time = ros::Time::now();
    m_get_global_trajectory = true;
    motion_mode = polytraj->motion_mode;
//    motion_mode = 8;
}


void LocalPlanner::init(ros::NodeHandle &nh, std::shared_ptr<GlobalMap_track> &_global_map)
{
    nh.param("tracking_node/local_v_max", m_vmax, 6.0);
    nh.param("tracking_node/local_a_max", m_amax, 6.0);
    nh.param("tracking_node/local_w_max", m_wmax, 8.0);
    nh.param("tracking_node/local_j_max", m_jmax, 8.0);
    nh.param("tracking_node/local_vxtl_max", m_vxtl_max, 1.8);
    nh.param("tracking_node/local_axtl_max", m_axtl_max, 2.0);
    nh.param("tracking_node/local_wxtl_max", m_wxtl_max, 4.0);
    nh.param("tracking_node/local_jxtl_max", m_jxtl_max, 3.0);
    nh.param("tracking_node/rho_", rho_, 1.0);
    nh.param("tracking_node/rhoN_", rhoN_, 2.0);
    nh.param("tracking_node/planning_horizon", planning_horizon, 20);
    nh.param("tracking_node/dt", dt, 0.1);
    nh.param("tracking_node/occ_file_path", occ_file_path, std::string("occfinal.png"));
    nh.param("tracking_node/taskFile", taskFile, std::string("task.info"));

    global_map = _global_map;

    cv::Mat occ_map3c;
    occ_map3c = cv::imread(occ_file_path);
    std::vector <cv::Mat> channels;
    cv::split(occ_map3c, channels);
    occ_map = channels.at(0);
    occ_map = swellOccMap(occ_map);

    ROS_WARN("m_vmax: %f", m_vmax);

    predictState.resize(planning_horizon);
    predictInput.resize(planning_horizon);
    ref_trajectory.resize(planning_horizon);
    for (int i = 0; i < planning_horizon; ++i) {
        predictInput[i].setZero();
    }

    reference_trajectory_sub = nh.subscribe("/trajectory_generation/global_trajectory", 1, &LocalPlanner::rcvGlobalTrajectory, this);
    redecision_pub = nh.advertise<std_msgs::Bool>("/redecide_flag", 1);
    mpcInterface_.init(taskFile);
//    mpcInterface_.obsConstraintPtr_.reset(new ObsConstraintSet(reference_time, obs_points));
    mpcInterface_.obsConstraintPtr_.reset(new ObsConstraintSet(reference_time, obs_points_t));
//    mpcSolverPtr_.reset(new ocs2::SqpMpc(mpcInterface_.mpcSettings(), mpcInterface_.sqpSettings(), mpcInterface_.getOptimalControlProblem(), mpcInterface_.getInitializer()));
    bufferPrimalSolutionPtr_.reset(new ocs2::PrimalSolution());
}

cv::Mat LocalPlanner::swellOccMap(cv::Mat occ_map)
{
    cv::Mat occ = cv::Mat::zeros(occ_map.rows, occ_map.cols, CV_8UC1);
    int swell_num = (int) (0.3 / 0.1);

    for (int i = 0; i < occ_map.rows; i++) {
        for (int j = 0; j < occ_map.cols; j++) {
            if (occ_map.at<uchar>(i, j) > 10){   // 膨胀直接画圆
                cv::circle(occ, cv::Point(j, i), swell_num, cv::Scalar(255, 255, 255), -1);
            }
        }
    }
    ROS_WARN("map swell done");
    return occ;
}

int LocalPlanner::solveNMPC(Eigen::Vector4d state)
{
    ocs2::scalar_array_t desiredTimeTrajectory(planning_horizon);
    ocs2::vector_array_t desiredStateTrajectory(planning_horizon);
    ocs2::vector_array_t desiredInputTrajectory(planning_horizon);
    for(size_t i = 0; i < planning_horizon; i++)  // 设置参考轨迹
    {

        Eigen::Vector4d reference_state = {ref_trajectory[i](0) + 0.0, ref_trajectory[i](1), ref_speed[i], ref_phi[i]};
        desiredStateTrajectory[i] = reference_state;
        desiredTimeTrajectory[i] = reference_time[i];
        desiredInputTrajectory[i] = Eigen::Vector2d::Zero(2);
    }
    ocs2::TargetTrajectories targetTrajectories(desiredTimeTrajectory, desiredStateTrajectory, desiredInputTrajectory);
    mpcInterface_.getReferenceManagerPtr()->setTargetTrajectories(targetTrajectories);
    mpcSolverPtr_->getSolverPtr()->setReferenceManager(mpcInterface_.getReferenceManagerPtr());
    state(2) = speed_direction * state(2);
//    std::cout<<"state(3): "<<state(3)<<std::endl;
    observation.state = state;
    bool controllerIsUpdated = mpcSolverPtr_->run(observation.time, observation.state);
    if (!controllerIsUpdated) {
        return 0;
    }
    ocs2::scalar_t final_time = observation.time + mpcSolverPtr_->settings().solutionTimeWindow_;
    mpcSolverPtr_->getSolverPtr()->getPrimalSolution(final_time, bufferPrimalSolutionPtr_.get());
    return 1;
}


bool LocalPlanner::lineVisib(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2)
{
    double ray_ptx, ray_pty, ray_ptz;
    double p2_x, p2_y, p2_z;
    p2_x = p2.x();
    p2_y = p2.y();
    p2_z = p2.z();

    double x_offset = p1.x() - p2.x();
    double y_offset = p1.y() - p2.y();

    double distance = std::sqrt(pow(x_offset, 2) + pow(y_offset, 2));

    int n = std::sqrt(pow(x_offset, 2) + pow(y_offset, 2)) / 0.1;
    for(int i = 0; i<n+1; i++)
    {
        ray_ptx = p2_x + i * 0.1 * x_offset / distance;
        ray_pty = p2_y + i * 0.1 * y_offset / distance;
        ray_ptz = 0.0;

        int idx, idy, idz;
        global_map->coord2gridIndex(ray_ptx, ray_pty, ray_ptz, idx, idy, idz);
        if (global_map->isOccupied(idx, idy, idz)){
            return false;
        }
    }
    return true;
}


void LocalPlanner::getNMPCPredictXU(Eigen::Vector4d &predict_state)
{
    predict_state(0) = bufferPrimalSolutionPtr_->stateTrajectory_[1](2);
    predict_state(1) = bufferPrimalSolutionPtr_->stateTrajectory_[1](3);
    predict_state(2) = bufferPrimalSolutionPtr_->inputTrajectory_[0](0);
    predict_state(3) = bufferPrimalSolutionPtr_->inputTrajectory_[0](1);
}


void LocalPlanner::getNMPCPredictionDeque(std::vector<Eigen::Vector4d> &state_deque, std::vector<Eigen::Vector2d> &input_deque)
{
    const size_t N = bufferPrimalSolutionPtr_->timeTrajectory_.size();
    for(int i = 0;i<N;i++)
    {
        state_deque.push_back(bufferPrimalSolutionPtr_->stateTrajectory_[i]);
//        std::cout<<"velocity: "<<bufferPrimalSolutionPtr_->stateTrajectory_[i][2]<<std::endl;
        input_deque.push_back(bufferPrimalSolutionPtr_->inputTrajectory_[i]);
        predictState.push_back(bufferPrimalSolutionPtr_->stateTrajectory_[i]);
        predictInput.push_back(bufferPrimalSolutionPtr_->inputTrajectory_[i]);
    }
}

bool LocalPlanner::checkfeasible()
{
    for(int i = 1; i<(predictState.size() * 0.75); i++){
        int idx, idy, idz;
        double pt_z = 0.0;
        global_map->coord2gridIndex(predictState[i](0), predictState[i](1), pt_z, idx, idy, idz);
        if (global_map->isOccupied(idx, idy, idz)){
            std::cout<<"occupied: "<<i<<std::endl;
            return true;
        }
    }
    return false;
}

bool LocalPlanner::checkBridge() {

    for(int i = 0; i<(predictState.size() - 1); i++) {
        int idx, idy, idz;
        double pt_z = 0.0;
        global_map->coord2gridIndex(predictState[i](0), predictState[i](1), pt_z, idx, idy, idz);
        if (global_map->GridNodeMap[idx][idy]->exist_second_height) {  // 在桥洞里不准转云台
            return true;
        }
    }
    return false;
}

bool LocalPlanner::checkXtl(){
    // 检查地形是否可以小陀螺
    double predictPathLength = 0.0;
    if(ref_trajectory.size() < 1){
        return true;
    }

    for(int i = 0; i < (predictState.size() - 1)/2; i++){
        int idx, idy, idz;
        double pt_z = 0.0;
        global_map->coord2gridIndex(predictState[i](0), predictState[i](1), pt_z, idx, idy, idz);
        if (global_map->GridNodeMap[idx][idy]->exist_second_height && global_map->GridNodeMap[idx][idy]->height < 0.4){  // 在桥洞里不准陀螺
            std::cout<<"in bridge"<<std::endl;
            return false;
        }


        Eigen::Vector3d sample_point, sample_point_next;
        sample_point.x() = predictState[i](0);
        sample_point.y() = predictState[i](1);
        sample_point_next.x() = predictState[i+1](0);
        sample_point_next.y() = predictState[i+1](1);

        Eigen::Vector3i sample_index = global_map->coord2gridIndex(sample_point);
        Eigen::Vector3i sample_index_next = global_map->coord2gridIndex(sample_point_next);
        double height_delta = global_map->GridNodeMap[sample_index.x()][sample_index.y()]->height -
                              global_map->GridNodeMap[sample_index_next.x()][sample_index_next.y()]->height;
        if(abs(height_delta) < 0.03){  // TODO 判断有点小问题
            continue;
        }

        predictPathLength = std::sqrt(std::pow(predictState[i+1](0) - predictState[i](0), 2) + std::pow(ref_trajectory[i+1](1) - ref_trajectory[i](1), 2));
        if(predictPathLength < 0.05){
            continue;
        }

        double slope = abs(height_delta) / predictPathLength;

        if(abs(slope) > 0.3){
            std::cout<<"in slope one"<<std::endl;
            return false;
        }
//        predictPathLength += std::sqrt(std::pow(predictState[i+1](0) - predictState[i](0), 2) + std::pow(predictState[i+1](1) - predictState[i](1), 2));

    }

    for(int i = 0; i < (predictState.size())/2; i++){
        int idx, idy, idz;
        double pt_z = 0.0;
        global_map->coord2gridIndex(predictState[i](0), predictState[i](1), pt_z, idx, idy, idz);
        if (global_map->GridNodeMap[idx][idy]->exist_second_height && global_map->GridNodeMap[idx][idy]->height < 0.4){  // 在桥洞里不准陀螺
            std::cout<<"in bridge"<<std::endl;
            return false;
        }


        Eigen::Vector3d sample_point, sample_point_next;
        sample_point.x() = predictState[i](0);
        sample_point.y() = predictState[i](1);
        sample_point_next.x() = predictState[i+4](0);
        sample_point_next.y() = predictState[i+4](1);

        Eigen::Vector3i sample_index = global_map->coord2gridIndex(sample_point);
        Eigen::Vector3i sample_index_next = global_map->coord2gridIndex(sample_point_next);
        double height_delta = global_map->GridNodeMap[sample_index.x()][sample_index.y()]->height -
                              global_map->GridNodeMap[sample_index_next.x()][sample_index_next.y()]->height;
        if(abs(height_delta) > 0.1){  // TODO 判断有点小问题
            return false;
        }
    }

    return true;
}

void LocalPlanner::getRefTrajectory()
{
    double time = accumulate(duration_time.begin(), duration_time.end(), 0.0);
    for(int i = 0; i<=(int)(time/dt); i++)
    {
        int index;
        double total_time;
        getSegmentIndex(i*dt, index, total_time);
//        std::cout<<" total_time: "<<total_time<<" i*dt: "<<i*dt<<" index: "<<index<<" time: "<<time<<std::endl;
//        std::cout<<"(i - (int)(total_time/dt)) * dt: "<< (i - (int)(total_time/dt)) * dt<<std::endl;

        Eigen::Vector3d ref_point;  //
        ref_point(0) = m_polyMatrix_x(index, 0) * pow(i*dt - total_time, 3) + m_polyMatrix_x(index, 1) * pow(
                i*dt - total_time, 2) + m_polyMatrix_x(index, 2) * (i*dt - total_time) + m_polyMatrix_x(
                index, 3);
        ref_point(1) = m_polyMatrix_y(index, 0) * pow(i*dt - total_time, 3) + m_polyMatrix_y(index, 1) * pow(
                i*dt - total_time, 2) + m_polyMatrix_y(index, 2) * (i*dt - total_time) + m_polyMatrix_y(
                index, 3);
        ref_point(2) = 0.0;
        reference_path.push_back(ref_point);
//        ROS_ERROR("reference_path[i].x() %f:  reference_path[i].y() %f: ", reference_path[i].x(), reference_path[i].y());
    }

    Eigen::Vector3d ref_point;  // (i - (int)(total_time/dt)) * dt：区段时间
    ref_point(0) = m_polyMatrix_x(duration_time.size() - 1, 0) * pow(duration_time.back(), 3) + m_polyMatrix_x(duration_time.size() - 1, 1) * pow(
            duration_time.back(), 2) + m_polyMatrix_x(duration_time.size() - 1, 2) * duration_time.back() + m_polyMatrix_x(
            duration_time.size() - 1, 3);
    ref_point(1) = m_polyMatrix_y(duration_time.size() - 1, 0) * pow(duration_time.back(), 3) + m_polyMatrix_y(duration_time.size() - 1, 1) * pow(
            duration_time.back(), 2) + m_polyMatrix_y(duration_time.size() - 1, 2) * duration_time.back() + m_polyMatrix_y(
            duration_time.size() - 1, 3);
    ref_point(2) = 0.0;
    reference_path.push_back(ref_point);
    target_point = reference_path.back();
}

void LocalPlanner::getRefVel()
{
    double time = accumulate(duration_time.begin(), duration_time.end(), 0.0);  /// 参考轨迹的时间分配
    for (int i = 0; i<=(int)(time/dt); i++)
    {
        int index;
        double total_time;
        getSegmentIndex(i*dt, index, total_time);
        Eigen::Vector3d ref_point;
        ref_point(0) = 3 * m_polyMatrix_x(index, 0) * pow((i*dt - total_time), 2) + 2 * m_polyMatrix_x(index, 1) * pow(
                (i*dt - total_time), 1) + m_polyMatrix_x(index, 2);
        ref_point(1) = 3 * m_polyMatrix_y(index, 0) * pow((i*dt - total_time), 2) + 2 * m_polyMatrix_y(index, 1) * pow(
                (i*dt - total_time), 1) + m_polyMatrix_y(index, 2);
        ref_point(2) = 0.0;
        reference_velocity.push_back(ref_point);
        ref_time.push_back(dt);
//        std::cout<<"vx: "<<ref_point(0)<<" vy: "<<ref_point(1)<<std::endl;
    }
//    Eigen::Vector3d ref_point;
//    ref_point(0) = 0.0;
//    ref_point(1) = 0.0;
//    ref_point(2) = 0.0;
//    reference_velocity.push_back(ref_point);
    ref_time.push_back(dt);
}

void LocalPlanner::getSegmentIndex(double time, int &segment_index, double &total_time)
{
    double sum_time = 0.0;
    for (int i = 0; i < duration_time.size(); i++) {
        sum_time += duration_time[i];
        if (sum_time >= time) {
            segment_index = i;   // 找到该时间对应的索引段和前几段的总时间
            total_time = sum_time - duration_time[i];
            break;
        }
    }
}

void LocalPlanner::getTrackingSegmentIndex(double time, int &segment_index, double &total_time)
{
    double sum_time = 0.0;
    for (int i = 0; i < ref_time.size(); i++) {
        sum_time += ref_time[i];
        if (sum_time >= time) {
            segment_index = i;   // 找到该时间对应的索引段和前几段的总时间
            total_time = sum_time - ref_time[i];
            break;
        }
        else
        {
            segment_index = ref_time.size();
            total_time = sum_time;
        }
    }
}

void LocalPlanner::getCircleEvadeTraj(Eigen::Vector3d cur_position, std::vector<Eigen::Vector3d> &ref_trajectory, std::vector<Eigen::Vector3d> &ref_velocity, std::vector<double> &reference_time){
    target_point = reference_path.back();  // 终点
    double theta = atan2((cur_position - target_point).y(), (cur_position - target_point).x());
    for(int i = 0; i < 20; i++){
        Eigen::Vector3d velocity_temp;
        Eigen::Vector3d position_temp;
        double delta_theta = (double) 0.1625; // (v*0.1/R)

        theta = theta + delta_theta;
        position_temp.x() = target_point.x() + 0.8 * cos(theta);
        position_temp.y() = target_point.y() + 0.8 * sin(theta);
        position_temp.z() = 0.0;

        velocity_temp.x() = 1.3 * sin(-theta);  // TODO 躲避障碍有点问题，终点判断可能有问题，以及巡航模式下的重规划
        velocity_temp.y() = 1.3 * cos(theta);
        velocity_temp.z() = 0.0;

        ref_trajectory.push_back(position_temp);
        ref_velocity.push_back(velocity_temp);

        reference_time.push_back(i * 0.1);

    }
}