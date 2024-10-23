#include "reference_path.hpp"
#include "algorithm"
#include "numeric"
void ReferenceSmooth::init(std::shared_ptr<GlobalMap>& global_map){
    global_map_= global_map;
}

ReferenceSmooth::~ReferenceSmooth(){

}

void ReferenceSmooth::setGlobalPath(Eigen::Vector3d velocity, std::vector<Eigen::Vector2d>& global_path, double reference_amax, double desire_speed, bool is_spinning)
{
    m_global_path = global_path;
    max_accleration = reference_amax;
    ROS_WARN("[Reference] path size : %d", global_path.size());
    state_vel = velocity;
    desire_veloity = desire_speed;
    is_spinning_ = is_spinning;
}
//求参考路径
void ReferenceSmooth::getRefTrajectory(std::vector<Eigen::Vector3d> &ref_trajectory, std::vector<double> &times)
{
    bool resolve = true;
    ref_trajectory.clear();
    reference_path.clear();//参考路径

    // 进行时间分配和三次多项式求解
    solveTrapezoidalTime();
    int iter = 0;
    while(resolve){
        if(iter < 3){
            solvePolyMatrix();
            resolve = checkfeasible();
            iter ++;
        }else{
            break;
        }
    }

    if (m_global_path.size() < 2) {
        traj_length = 0.0;
        ROS_ERROR("[Reference] global_path size < 2, stop solving！");
        return;
    }

    double time = std::accumulate(m_trapezoidal_time.begin(), m_trapezoidal_time.end(), 0.0);

    for (int i = 0; i<(int)(time / dt); i++)
    {
        int index;
        double total_time;
        getSegmentIndex(i * dt, index, total_time);
        Eigen::Vector3d ref_point;
        ref_point(0) = m_polyMatrix_x(index, 0) * pow((i * dt - total_time), 3) + m_polyMatrix_x(index, 1) * pow(
                (i * dt - total_time), 2) + m_polyMatrix_x(index, 2) * (i - (int) (total_time / dt)) * dt + m_polyMatrix_x(
                index, 3);
        ref_point(1) = m_polyMatrix_y(index, 0) * pow((i * dt - total_time), 3) + m_polyMatrix_y(index, 1) * pow(
                (i * dt - total_time), 2) + m_polyMatrix_y(index, 2) * (i - (int) (total_time / dt)) * dt + m_polyMatrix_y(
                index, 3);
        ref_point(2) = 0.0;
        ref_trajectory.push_back(ref_point);
        reference_path.push_back(ref_point);
    }
}

void ReferenceSmooth::getRefVel()
{
    reference_velocity.clear();
    double time = accumulate(m_trapezoidal_time.begin(), m_trapezoidal_time.end(), 0.0);  /// 参考轨迹的时间分配
    for (int i = 0; i<(int)(time/dt); i++)
    {
        int index;
        double total_time;
        getSegmentIndex(i*dt, index, total_time);
        Eigen::Vector3d ref_point;
        ref_point(0) = 3 * m_polyMatrix_x(index, 0) * pow((i - (int) (total_time / dt)) * dt, 2) + 2 * m_polyMatrix_x(index, 1) * pow(
                (i - (int) (total_time / dt)) * dt, 1) + m_polyMatrix_x(index, 2);
        ref_point(1) = 3 * m_polyMatrix_y(index, 0) * pow((i - (int) (total_time / dt)) * dt, 2) + 2 * m_polyMatrix_y(index, 1) * pow(
                (i - (int) (total_time / dt)) * dt, 1) + m_polyMatrix_y(index, 2);
        ref_point(2) = 0.0;
        reference_velocity.push_back(ref_point);
    }
}

void ReferenceSmooth::solveTrapezoidalTime(){
    double path_length = 0;
    if(m_global_path.size()<2){
        traj_length = 0.0;
        ROS_ERROR("[Reference] global_path size < 2, No path");
        return;
    }
    m_trapezoidal_time.clear();
    for(int i = 0; i < m_global_path.size() - 1; i++)
    {
        double pz = 0.0;
        int idx, idy, idz, idx_end, idy_end, idz_end;
        //二维查找
        global_map_->coord2gridIndex(m_global_path[i].x(), m_global_path[i].y(), pz, idx, idy, idz);
        global_map_->coord2gridIndex(m_global_path[i+1].x(), m_global_path[i+1].y(), pz, idx_end, idy_end, idz_end);
        double path_distance = (double)std::sqrt(pow(m_global_path[i+1].x() - m_global_path[i].x(), 2) + pow(m_global_path[i+1].y() - m_global_path[i].y(), 2));
        double height = global_map_->GridNodeMap[idx_end][idy_end]->height_;
        //高度上的斜率
        double slope = abs(height - global_map_->GridNodeMap[idx][idy]->height_) / path_distance;
        if(height > 0.0){
        //std::cout<<"slope: "<<abs(height - global_map->GridNodeMap[idx][idy]->height)/ path_distance<<std::endl;
            std::cout << "ratio: " << path_distance / 0.3 << std::endl;
        }

        int sample_num = path_distance / 0.3;  // 采样间隔0.3来确定坡道
        bool exist_height_change = false;
        for(int j = 0; j <= sample_num; j++){
            Eigen::Vector3d sample_point;
            sample_point.x() = m_global_path[i].x() + 0.3 * j * (m_global_path[i+1].x() - m_global_path[i].x()) / path_distance;
            sample_point.y() = m_global_path[i].y() + 0.3 * j * (m_global_path[i+1].y() - m_global_path[i].y()) / path_distance;
            Eigen::Vector3i sample_index = global_map_->coord2gridIndex(sample_point);
//            if(abs(height - global_map->GridNodeMap[idx][idy]->height) > 0.05){   // 判断是否存在高度变化 TODO 暂时设置为false，即为上坡减速
//                exist_height_change = true;
//            }
            //查看是否存在高度变化，下坡
            if((global_map_->GridNodeMap[idx][idy]->height_ - height) > 0.05){   // 判断是否存在高度变化 TODO 暂时设置为false，即为上坡减速
                exist_height_change = true;
            }
        }

        double slope_cof = slope_coeff;
        double bridge_cof = bridge_coeff;
        double slope_cof_max = slope_coeff_max;

        if(is_spinning_){
            slope_cof = slope_coeff_spinning;
            bridge_cof = bridge_coeff_spinning;
            slope_cof_max = slope_coeff_spinning_max;
        }

        path_length += path_distance;
        traj_length = path_length;
        
        //每一步0.3m
        double time = 0.3 / desire_veloity;
        if(global_map_->GridNodeMap[idx][idy]->exist_second_height_ ||
           global_map_->GridNodeMap[idx_end][idy_end]->exist_second_height_){
            time = time * bridge_cof;  // 如果存在过桥洞的段直接减速！
        }else if(exist_height_change){  // 只针对下坡减速
            double ratio = (slope * slope_cof) > slope_cof_max ? slope_cof_max: (slope * slope_cof);
            time = time * (1.0 + ratio);
        }
        m_trapezoidal_time.push_back(time);
    }
    //    for(int i = 0; i < m_global_path.size() - 1; i++){
    //        std::cout<<"m_trapezoidal_time: "<<m_trapezoidal_time[i]<<std::endl;
    //    }
    ROS_WARN("[Reference] path_length: %f", path_length);
}


void ReferenceSmooth::solvePolyMatrix()
{
    if (m_global_path.size() < 2) {
        ROS_ERROR("[Reference] global_path size < 2, stop solving！");
        return;
    }

    reset(m_global_path.size() - 1);
    m_bandedMatrix.reset();
    m_bandedMatrix(0, 0) = 2 * m_trapezoidal_time[0];
    m_bandedMatrix(0, 1) = m_trapezoidal_time[0];
    for(int i = 1 ;i < N ;i++)
    {
        m_bandedMatrix(i, i - 1) = m_trapezoidal_time[i - 1];
        m_bandedMatrix(i, i) = 2 * (m_trapezoidal_time[i - 1] + m_trapezoidal_time[i]);
        m_bandedMatrix(i, i + 1) = m_trapezoidal_time[i];
    }

    m_bandedMatrix(N, N - 1) = m_trapezoidal_time[N - 1];
    m_bandedMatrix(N, N) = 2 * m_trapezoidal_time[N - 1];

    b(0, 0) = 6 * (((m_global_path[1].x() - m_global_path[0].x()) / m_trapezoidal_time[0]) - state_vel[0]);
    b(N, 0) = -6 * ((m_global_path[N].x() - m_global_path[N - 1].x()) / m_trapezoidal_time[N - 1]);
    //    b(0, 0) = 0.0;
    //    b(N, 0) = 0.0;

    for(int i = 1; i < N; i++)
    {
        b(i, 0) = 6 * ((m_global_path[i + 1].x() - m_global_path[i].x()) / m_trapezoidal_time[i] - (m_global_path[i].x() - m_global_path[i - 1].x()) / m_trapezoidal_time[i - 1]);
    }

    b2(0, 0) = 6 * (((m_global_path[1].y() - m_global_path[0].y()) / m_trapezoidal_time[0]) - state_vel[1]);
    b2(N, 0) = -6 * ((m_global_path[N].y() - m_global_path[N - 1].y()) / m_trapezoidal_time[N - 1]);

    //    b2(0, 0) = 0.0;
    //    b2(N, 0) = 0.0;

    for (int i = 1; i < N; i++) {
        b2(i, 0) = 6 * ((m_global_path[i + 1].y() - m_global_path[i].y()) / m_trapezoidal_time[i] -
                       (m_global_path[i].y() - m_global_path[i - 1].y()) / m_trapezoidal_time[i - 1]);
    }

    m_bandedMatrix.factorizeLU();
    m_bandedMatrix.solve(b);
    m_bandedMatrix.solve(b2);


    // 解得的中间变量矩阵m1m2,用于求解系数矩阵 y = a + bx + cx2 + dx3
    for(int i = 0; i < N; i++){
        m_polyMatrix_x(i, 0) = (b(i + 1, 0) - b(i, 0)) / (6 * m_trapezoidal_time[i]);
        m_polyMatrix_x(i, 1) = b(i, 0) / 2;
        m_polyMatrix_x(i, 2) = (m_global_path[i + 1].x() - m_global_path[i].x()) / m_trapezoidal_time[i] - m_trapezoidal_time[i] * (2 * b(i, 0) + b(i + 1, 0)) / 6;
        m_polyMatrix_x(i, 3) = m_global_path[i].x();

        m_polyMatrix_y(i, 0) = (b2(i + 1, 0) - b2(i, 0)) / (6 * m_trapezoidal_time[i]);
        m_polyMatrix_y(i, 1) = b2(i, 0) / 2;
        m_polyMatrix_y(i, 2) = (m_global_path[i + 1].y() - m_global_path[i].y()) / m_trapezoidal_time[i] -
                               m_trapezoidal_time[i] * (2 * b2(i, 0) + b2(i + 1, 0)) / 6;
        m_polyMatrix_y(i, 3) = m_global_path[i].y();
    }
    return;
}


void ReferenceSmooth::reset(const int &pieceNum) {
    N = pieceNum;
    m_bandedMatrix.create(N + 1, 1, 1);
    b.resize(N + 1, 1);
    b2.resize(N + 1, 1);
    m_polyMatrix_x.resize(N, 4);
    m_polyMatrix_y.resize(N, 4);
    return;
}

bool ReferenceSmooth::checkfeasible()
{
    bool resolve = false;
    std::vector<bool> ischecked(m_trapezoidal_time.size(), false);
    double time = std::accumulate(m_trapezoidal_time.begin(), m_trapezoidal_time.end(), 0.0);  /// 参考轨迹的时间分配
    //ROS_ERROR("m_global_path size: %f", time);
    std::vector<double> m_trapezoidal_time_temp = m_trapezoidal_time;
    for (int i = 0; i<(int)(time / dt); i++)
    {
        int index;
        double total_time;
        double accleration;
        getSegmentIndex(i * dt, index, total_time);
        Eigen::Vector3d ref_point;
        ref_point(0) = 6 * m_polyMatrix_x(index, 0) * pow((i * dt - total_time), 1) + 2 * m_polyMatrix_x(index, 1);
        ref_point(1) = 6 * m_polyMatrix_y(index, 0) * pow((i * dt - total_time), 1) + 2 * m_polyMatrix_y(index, 1);
        ref_point(2) = 0.0;
        //计算加速度
        accleration = sqrt(pow(ref_point(0), 2) + pow(ref_point(1), 2));
        //std::cout<<"time: "<<i * dt - total_time<<" index: "<<index<<" acc x: "<<ref_point(0)<<" acc y: "<<ref_point(1)<<std::endl;
        //加速度过大并且还没有检查
        if(accleration > max_accleration && ischecked[index] == false)
        {
            if(index > 0 && index < m_trapezoidal_time.size() - 1){
                //适当延长时间
                m_trapezoidal_time_temp[index - 1] = m_trapezoidal_time_temp[index - 1] * 1.1;
                m_trapezoidal_time_temp[index + 1] = m_trapezoidal_time_temp[index + 1] * 1.1;
                ischecked[index - 1] = true;
                ischecked[index + 1] = true;
            } else if(index == 0){
                m_trapezoidal_time_temp[index + 1] = m_trapezoidal_time_temp[index + 1] * 1.1;
                ischecked[index + 1] = true;
            } else if(index == m_trapezoidal_time.size() - 1){
                m_trapezoidal_time_temp[index - 1] = m_trapezoidal_time_temp[index - 1] * 1.1;
                ischecked[index - 1] = true;
            }
            m_trapezoidal_time_temp[index] = m_trapezoidal_time_temp[index] * 1.1;
            ischecked[index] = true;
            //只要有较大的加速度那么就进行修正
            resolve = true;
        }

    }
    m_trapezoidal_time = m_trapezoidal_time_temp;
    return resolve;
}

void ReferenceSmooth::getSegmentIndex(double time, int &segment_index, double &total_time)
{
    double sum_time = 0.0;
    for (int i = 0; i < m_trapezoidal_time.size(); i++) {
        sum_time += m_trapezoidal_time[i];
        if (sum_time >= time) {
            segment_index = i;
            total_time = sum_time - m_trapezoidal_time[i];
            break;
        }
    }
}


