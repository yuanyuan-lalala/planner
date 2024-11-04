#include "optimizer/cubic_spline_optimizer.hpp"

CubicSplineOptimizer::CubicSplineOptimizer(std::shared_ptr<GlobalMap> global_map):OptimizerAlgorithm(global_map){

}

void CubicSplineOptimizer::init(std::vector<Eigen::Vector3d>& global_path, Eigen::Vector3d start_vel, double desire_speed)
{
    //每隔0.3m采一个点
    pathSample(global_path, start_vel);
    if(m_sampled_path.size() < 2){
        ROS_ERROR("[Smooth Init] path.size()<2!!  No path");
        return;
    }
    desire_velocity = desire_speed;
    headP = m_sampled_path[0];
    
    Eigen::Vector2d start_vels;
    start_vels.x() = start_vel.x();
    start_vels.y() = start_vel.y();
    double radius = 0.5;
    getGuidePath(start_vels * 10,radius);
    
    if(!init_vel){  // 速度方向反向过大直接倒车不拐弯
        start_vels.x() = 0.0;
        start_vels.y() = 0.0;
    }
    tailP = m_sampled_path[m_sampled_path.size() - 1];
    pieceN = m_sampled_path.size() - 1;
    cubSpline.setConditions(headP, start_vels, tailP, pieceN);
}

void CubicSplineOptimizer::pathSample(std::vector<Eigen::Vector3d>& global_path, Eigen::Vector3d start_vel)
{
    std::vector<Eigen::Vector2d> trajectory_point;
    double step = 0.3;
    double last_dis = 0.0;  /// 上一段留下来的路径
    m_sampled_path.clear();
    trajectory_point.clear();
    if(global_path.size() < 2){
        ROS_ERROR("[Smooth Sample] global_path.size()<2, No path");
        return;
    }
    for (int i = 0;i < global_path.size() - 1;i++)
    {
        Eigen::Vector2d start(global_path[i].x(), global_path[i].y());
        Eigen::Vector2d end(global_path[i+1].x(), global_path[i+1].y());
        Eigen::Vector2d start2end(global_path[i+1].x() - global_path[i].x(), global_path[i+1].y() - global_path[i].y());
        double path_distance = std::sqrt(pow(start.x() - end.x(), 2) + pow(start.y() - end.y(), 2));
        
     
        if(trajectory_point.empty()){
            trajectory_point.push_back(start);
        }
        /// 将上一段路径残留的部分放进去
        Eigen::Vector2d start_new;
        if(path_distance >= (step - last_dis)){
            start_new.x() = start.x() + start2end.x() * (step - last_dis)/path_distance;
            start_new.y() = start.y() + start2end.y() * (step - last_dis)/path_distance;
        
            step = 0.3;  // 无论速度多大我们只调整第一段的长度，push_back后直接回归低采样
        }
        else  /// 这段路径太短了，加上last_dis还没有采样步长长，因此直接加到dis_last中
        {
            last_dis = last_dis + path_distance;  /// TODO 这种处理方式有个问题，容易把角点扔掉，不保证安全性
            continue;
        }

        Eigen::Vector2d new_start2end(end.x() - start_new.x(), end.y() - start_new.y());
        
        double path_distance_new = std::sqrt(pow(start_new.x() - end.x(), 2) + pow(start_new.y() - end.y(), 2));
        int sample_num = (int)((path_distance_new)/step);
        if(path_distance_new < 0.1){
            last_dis = (path_distance-(step-last_dis)) - ((float)sample_num) * step;
            trajectory_point.push_back(start_new);
            continue;
        }

        for (int j = 0;j<=sample_num; j++){
            Eigen::Vector2d sample_point(start_new.x() + new_start2end.x() * step/path_distance_new * j,
                            start_new.y() + new_start2end.y() * step/path_distance_new * j);
            trajectory_point.push_back(sample_point);
        }
        last_dis = (path_distance-(step-last_dis)) - ((float)sample_num) * step;
    }

    if(last_dis < step/2.0f && trajectory_point.size()>=2)
    {
        trajectory_point.erase((trajectory_point.end()-1));
    }
    Eigen::Vector2d end(global_path[global_path.size() - 1].x(),global_path[global_path.size() - 1].y());
    trajectory_point.push_back(end);
    //采样获得的初始轨迹
    m_sampled_path.assign(trajectory_point.begin(), trajectory_point.end());
}

void CubicSplineOptimizer::getGuidePath(Eigen::Vector2d start_vel, double radius){
    if(m_sampled_path.size()<3){
        ROS_ERROR("[Smooth Path] path.size()<3, No need to optimize");
        return;
    }
    Eigen::Vector2d direction = m_sampled_path[1] - m_sampled_path[0];
    Eigen::Vector2d start_point = m_sampled_path[0];
    direction = direction / direction.norm();

    double theta = acos((start_vel.x() * direction.x() + start_vel.y() * direction.y()) / (direction.norm() * start_vel.norm()));
    //夹角在120度内并且初始速度在0.2m/s时认为速度初始化正确
    if(theta < M_PI_2 * 4/3 && start_vel.norm() > 0.2){
        init_vel = true;
    }else{
        init_vel = false;
    }
}

void CubicSplineOptimizer::optimize(){

    init(m_unoptimized_path,m_start_vel,m_reference_speed);
    optimizePath();
    pathResample();
    // m_final_path = getPath(); 
 

}


void CubicSplineOptimizer::optimizePath()
{
    m_global_map->allobs.clear();
    mid_distance.clear();
    init_obs = false;   
    init_vel = false;  // 是否初始化速度
    using_curv = false;  // 是否启用曲率优化
    // TODO 时间维度N 空间维度2(N-1)
    if(m_sampled_path.size()<3){
        ROS_ERROR("[Smooth Path] path.size()<3, No need to optimize");
        return;
    }
    Eigen::VectorXd x(pieceN * 2 - 2);

    for(int i = 0; i < pieceN - 1; i++){  // 控制点
        x(i) = m_sampled_path[i + 1].x();
        x(i + pieceN - 1) = m_sampled_path[i + 1].y();
    }

    m_global_map->obs_coord.clear();

    double minCost = 0.0;
    lbfgs_params.mem_size = 64;  // 32
    lbfgs_params.past = 5;
    lbfgs_params.min_step = 1.0e-32;
    lbfgs_params.g_epsilon = 2.0e-5;
    lbfgs_params.delta = 2e-5;  // 3e-4
    lbfgs_params.max_linesearch = 32;  // 32
    lbfgs_params.f_dec_coeff = 1.0e-4;
    lbfgs_params.s_curv_coeff = 0.9;

    int ret = lbfgs::lbfgs_optimize(x,
                                    minCost,
                                    &CubicSplineOptimizer::costFunction,
                                    nullptr,
                                    this,
                                    lbfgs_params);

    if (ret >= 0 || ret == lbfgs::LBFGSERR_MAXIMUMLINESEARCH)
    {
        if(ret > 0){
            //优化成功
            ROS_DEBUG_STREAM("[Smooth Optimize] Optimization Success: "
                             << lbfgs::lbfgs_stderr(ret));
        }else if (ret == 0){
            //优化停止
            ROS_INFO_STREAM("[Smooth Optimize] Optimization STOP: "
                                     << lbfgs::lbfgs_stderr(ret));
        }
        else{
            ROS_INFO_STREAM("[Smooth Optimize] Optimization reaches the maximum number of evaluations: "
                                    << lbfgs::lbfgs_stderr(ret));
        }
        for(int i = 0; i < pieceN - 1; i++){
            m_sampled_path[i + 1].x() = x(i);
            m_sampled_path[i + 1].y() = x(i + pieceN - 1);
        }
        pathInPs.resize(2, pieceN - 1);
        pathInPs.row(0) = x.head(pieceN - 1);
        pathInPs.row(1) = x.segment(pieceN - 1, pieceN - 1);
    }
    else
    {
        m_sampled_path.clear();
        ROS_ERROR_STREAM("[Smooth Optimize] Optimization Failed: "
                                << lbfgs::lbfgs_stderr(ret));
    }
}

void CubicSplineOptimizer::pathResample()
{
    ROS_INFO("[Smooth Resample] resample");
    m_final_path.clear();
    std::vector<Eigen::Vector2d> trajectory_point;
    const int step = 2;
    trajectory_point.clear();
    if(m_sampled_path.size()<2){
        ROS_ERROR("[Smooth Resample] path Resample.size()<3, No path");
        return;
    }
    trajectory_point.push_back(m_sampled_path[0]);
    int sample_num = int((m_sampled_path.size() - 1)/step);
    for (int i = 1; i < sample_num; i++){
        trajectory_point.push_back(m_sampled_path[i*step]);
    }
    if(int(m_sampled_path.size() -1) > step*sample_num){
        trajectory_point.push_back((m_sampled_path.back()));
    }
    m_final_path.assign(m_sampled_path.begin(), m_sampled_path.end());
}

std::vector<Eigen::Vector2d> CubicSplineOptimizer::getPath()
{

    return m_final_path;
}

double CubicSplineOptimizer::costFunction(void *ptr, const Eigen::VectorXd &x, Eigen::VectorXd &g)
{
    clock_t time_1 = clock();

    auto instance = reinterpret_cast<CubicSplineOptimizer *>(ptr);
    const int points_num = instance->pieceN - 1;
    Eigen::Matrix2Xd grad;
    grad.resize(2, points_num);
    grad.setZero();
    double cost = 0.0;

    Eigen::Matrix2Xd inPs;
    inPs.resize(2, points_num);
    inPs.row(0) = x.head(points_num);  // 扁平化输入inPs, x是xy所有的数据拉成了一维的形式
    inPs.row(1) = x.tail(points_num);
    
    Eigen::VectorXd inTimes;
    inTimes.resize(instance->pieceN);

    for(int i = 0; i < instance->pieceN; i++){
        //给相同的时间
        inTimes(i) = 0.3 / instance->desire_velocity;
    }
    instance->cubSpline.setInnerPoints(inPs, inTimes);

    if(!instance->init_obs) {
        for (int i = 0; i < points_num; ++i) {
            double nearest_cost = 0.0;
            Eigen::Vector2d x_cur = inPs.col(i);
            Eigen::Vector2d obstacleGrad = instance->obstacleTerm(i, x_cur, nearest_cost);
        }
    }
    instance->init_obs = true;
    /// 整个优化过程最最最耗时的地方
    //存储与时间和控制点相关的能量梯度
    double energy;
    Eigen::Matrix2Xd energy_grad;
    Eigen::VectorXd energyT_grad, energyTau_grad;
    energy_grad.resize(2, points_num);
    energy_grad.setZero();
    energyT_grad.resize(instance->pieceN);
    energyT_grad.setZero();
    energyTau_grad.resize(instance->pieceN);
    energyTau_grad.setZero();

    instance->cubSpline.getEnergy(energy);  // 能量损失cost和梯度
    instance->cubSpline.getGradSmooth(energy_grad, energyT_grad);
    cost += energy;
    grad += energy_grad;

    Eigen::Matrix2Xd potential_grad;
    potential_grad.resize(2, points_num);
    potential_grad.setZero();
    for(int i = 0; i<points_num; ++i){
        double nearest_cost = 0.0;
        Eigen::Vector2d x_cur = inPs.col(i);
        Eigen::Vector2d obstacleGrad = instance->obstacleTerm(i, x_cur, nearest_cost);

        potential_grad.col(i) = obstacleGrad;
        cost += nearest_cost;
    }
    clock_t time_2 = clock();
    grad += potential_grad;

    g.setZero();
    // 控制点和时间梯度
    g.head(points_num) = grad.row(0).transpose();
    g.tail(points_num) = grad.row(1).transpose();
    clock_t time_3 = clock();
    return cost;
}

Eigen::Vector2d CubicSplineOptimizer::obstacleTerm(int idx, Eigen::Vector2d xcur, double &nearest_cost)
{
    nearest_cost = 0.0;
    Eigen::Vector2d gradient(0.0, 0.0);
    double R = 0.3;
    if(!init_obs)
    {
        //没初始化的初始化一下障碍物
        m_global_map->obs_coord.clear();
        
        m_global_map->getObsEdge(xcur);
        m_global_map->allobs.push_back(m_global_map->obs_coord);


        std::vector<double> distance_temp;
        double ddd = 0.0;
        for(int i = 0; i < m_global_map->obs_coord.size(); i++)
        {
            double distance = std::sqrt((xcur(0) - m_global_map->obs_coord[i](0)) * (xcur(0) - m_global_map->obs_coord[i](0)) +
                                        (xcur(1) - m_global_map->obs_coord[i](1)) * (xcur(1) - m_global_map->obs_coord[i](1)));
            //距离太离谱的让他滚
            if(distance > (R * 1.2)){
                continue;
            }
            distance_temp.push_back(distance);
            ddd += distance;
        }

        if(!distance_temp.empty()){
            mid_distance.push_back(ddd/distance_temp.size());
        }else{
            mid_distance.push_back(0.3);
        }

        return gradient;
    }
    int size = m_global_map->allobs[idx].size();
    double alpha = 1.0;
    for(int i = 0; i < m_global_map->allobs[idx].size(); i++)
    {

        double tempR = mid_distance[idx];//每个障碍物点集的平均
        alpha = 1.0;
        double distance = std::sqrt((xcur(0) - m_global_map->allobs[idx][i](0)) * (xcur(0) - m_global_map->allobs[idx][i](0)) 
                            + (xcur(1) - m_global_map->allobs[idx][i](1)) * (xcur(1) - m_global_map->allobs[idx][i](1)));
        if(distance > tempR || distance < 1e-10){
            continue;
        }
        // xcur 越靠近障碍物时，distance 变小，而 (tempR - distance) 变大
        nearest_cost += pow(alpha, 2) * m_global_map->wObstacle * (tempR - distance) * (tempR - distance)  / (double)size;
        Eigen::Vector2d obsVct(xcur.x() - m_global_map->allobs[idx][i](0), xcur.y() - m_global_map->allobs[idx][i](1));
        // 当 distance 小于 tempR 时，distance - tempR 为负值，表示 xcur 在障碍物的影响范围内。这个负值通过梯度使得 xcur 产生远离障碍物的趋势。
        // 乘以 2 是为了将梯度计算中的平方项的系数还原到一阶导数中。
        // 将每个障碍物的梯度分量平均化，以确保梯度的强度与障碍物数量无关，便于在多障碍物情况下对 xcur 的位置进行平衡调整。
        gradient += pow(alpha, 1) * m_global_map->wObstacle * 2 * (obsVct / distance) * (distance - tempR) / (double)size;
    }
    return gradient;
}
