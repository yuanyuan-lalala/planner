#include"map/base_map.hpp"

Eigen::Vector3d BaseMap::gridIndex2coord(const Eigen::Vector3i &index)
{
    Eigen::Vector3d pt;

    pt(0) = ((double)index(0) + 0.5) * m_resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * m_resolution + gl_yl;
    pt(2) = ((double)index(2) + 0.5) * m_resolution + gl_zl;

    return pt;
}

Eigen::Vector3i BaseMap::coord2gridIndex(const Eigen::Vector3d &pt) {
    Eigen::Vector3i idx;
    idx << std::min(std::max(int((pt(0) - gl_xl) * m_inv_resolution), 0), GLX_SIZE - 1),
            std::min(std::max(int((pt(1) - gl_yl) * m_inv_resolution), 0), GLY_SIZE - 1),
            std::min(std::max(int((pt(2) - gl_zl) * m_inv_resolution), 0), GLZ_SIZE - 1);

    return idx;
}

void BaseMap::coord2gridIndex(double &pt_x, double &pt_y, double &pt_z, int &x_idx, int &y_idx, int &z_idx)
{
    x_idx = std::min(std::max(int((pt_x - gl_xl) * m_inv_resolution), 0), GLX_SIZE - 1);
    y_idx = std::min(std::max(int((pt_y - gl_yl) * m_inv_resolution), 0), GLY_SIZE - 1);
    z_idx = std::min(std::max(int((pt_z - gl_zl) * m_inv_resolution), 0), GLZ_SIZE - 1);
}


void BaseMap::setRadiusDash(const double dash_radius)
{
    m_robot_radius_dash = dash_radius;
}

bool BaseMap::isOccupied(const Eigen::Vector3i &index, bool second_height) const
{
    return isOccupied(index(0), index(1), index(2), second_height);
}

bool BaseMap::isOccupied(const int &idx_x, const int &idx_y, const int &idx_z, bool second_height) const
{
    // 检查索引是否在有效范围内
    if (idx_x < 0 || idx_x >= GLX_SIZE || idx_y < 0 || idx_y >= GLY_SIZE || idx_z < 0 || idx_z >= GLZ_SIZE) {
        return false;
    }

    // 计算一维索引
    int index = idx_x * GLY_SIZE + idx_y;

    // 检查是否越界访问
    if (index < 0 || index >= m_global_obstacle_map.size() || index >= m_local_obstacle_map.size()) {
        std::cerr << "Index out of bounds: " << index << std::endl;
        return false;
    }

    // 返回是否被占据，静态地图或动态地图任意一个为1即返回true
    return (m_global_obstacle_map[index] == 1 || m_local_obstacle_map[index] == 1);
}


bool BaseMap::isLocalOccupied(const Eigen::Vector3i &index) const
{
    return isLocalOccupied(index(0), index(1), index(2));
}

bool BaseMap::isLocalOccupied(const int &idx_x, const int &idx_y, const int &idx_z) const
{
    // 检查索引是否在有效范围内
    if (idx_x < 0 || idx_x >= GLX_SIZE || idx_y < 0 || idx_y >= GLY_SIZE || idx_z < 0 || idx_z >= GLZ_SIZE) {
        return false;
    }

    // 计算一维索引
    int index = idx_x * GLY_SIZE + idx_y;

    // 返回局部地图是否被占据
    return (m_local_obstacle_map[index] == 1);
}



cv::Mat BaseMap::swellOccMap(cv::Mat occ_map1c)
{
    cv::Mat occ = cv::Mat::zeros(occ_map1c.rows, occ_map1c.cols, CV_8UC1);
    int swell_num = (int) (m_robot_radius * m_inv_resolution);
    //    int swell_num = 4;
    for (int i = 0; i < occ_map1c.rows; i++) {
        for (int j = 0; j < occ_map1c.cols; j++) {
            if (occ_map1c.at<uchar>(i, j) > 10)
            {   // 膨胀直接画圆
                cv::circle(occ, cv::Point(j, i), swell_num, cv::Scalar(255, 255, 255), -1);
            }
        }
    }
    ROS_WARN("map swell done");
    return occ;
}

bool BaseMap::isFree(const Eigen::Vector3i &index) const
{
    return isFree(index(0), index(1), index(2));
}

bool BaseMap::isFree(const int &idx_x, const int &idx_y, const int &idx_z) const
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE &&
            (m_global_obstacle_map[idx_x * GLY_SIZE + idx_y] < 1 && m_local_obstacle_map[idx_x * GLY_SIZE + idx_y] < 1));
}
void BaseMap::resetUsedGrids()
{
    for (int i = 0; i < GLX_SIZE; i++)
        for (int j = 0; j < GLY_SIZE; j++)
        {
            resetGrid((*GridNodeMap)[i][j]);
        }
}

inline void BaseMap::resetGrid(GridNodePtr ptr)
{
    ptr->m_set_type = GridNode::setType::UNEXPLORE;
    ptr->m_cameFrom = nullptr;
    ptr->m_gScore = inf;
    ptr->m_fScore = inf;
}

Eigen::Vector3d BaseMap::coordRounding(const Eigen::Vector3d &coord)
{
    return gridIndex2coord(coord2gridIndex(coord));
}

double BaseMap::getHeight(const int idx_x, const int idx_y)
{
     if (idx_x > GLX_SIZE || idx_y > GLY_SIZE)
         return 0.0;
     return (*GridNodeMap)[idx_x][idx_y]->m_height;
}
void BaseMap::setHeight(const double coord_x, const double coord_y, const double coord_z, const bool exist_heights)
{
    // 检查坐标是否超出地图边界
    if (coord_x < gl_xl || coord_y < gl_yl || coord_x >= gl_xu || coord_y >= gl_yu) {
        std::cerr << "Coordinate out of bounds: (" << coord_x << ", " << coord_y << ")" << std::endl;
        return;
    }

    // 用障碍物空间点坐标找到其所对应的栅格索引
    int idx_x = static_cast<int>((coord_x - gl_xl + 0.01) * m_inv_resolution);
    int idx_y = static_cast<int>((coord_y - gl_yl + 0.01) * m_inv_resolution);

    // 打印索引和地图信息，帮助调试
    std::cout << "idx_x: " << idx_x << ", idx_y: " << idx_y << std::endl;
    std::cout << "GridNodeMap size: " << GridNodeMap->size() << std::endl;
    if (idx_x < GridNodeMap->size()) {
        std::cout << "Row size: " << (*GridNodeMap)[idx_x].size() << std::endl;
    }

    // 检查idx_x和idx_y是否在GridNodeMap的有效范围内
    if (idx_x < 0 || idx_x >= GridNodeMap->size() || 
        idx_y < 0 || idx_y >= (*GridNodeMap)[idx_x].size()) {
        std::cerr << "Index out of bounds: idx_x = " << idx_x << ", idx_y = " << idx_y << std::endl;
        return;
    }

    // 判断是否存在高度数据
    if (exist_heights) {
        // 设置高度值
        (*GridNodeMap)[idx_x][idx_y]->m_height = coord_z;
    }
}

// void BaseMap::setHeight(const double coord_x, const double coord_y, const double coord_z, const bool exist_heights)
// {
//     if (coord_x < gl_xl || coord_y < gl_yl ||
//         coord_x >= gl_xu || coord_y >= gl_yu )
//         return;

//     /* 用障碍物空间点坐标找到其所对应的栅格 */
//     int idx_x = static_cast<int>((coord_x - gl_xl + 0.01) * m_inv_resolution);
//     int idx_y = static_cast<int>((coord_y - gl_yl + 0.01) * m_inv_resolution);
//     std::cout << "idx_x: " << idx_x << ", idx_y: " << idx_y << std::endl;
//     std::cout << "GridNodeMap size: " << GridNodeMap->size() << std::endl;
//     std::cout << "Row size: " << (*GridNodeMap)[idx_x].size() << std::endl;
//     (*GridNodeMap)[idx_x][idx_y]->m_height = coord_z;
    
// }



void BaseMap::setObs(const double coord_x, const double coord_y)
{
    if (coord_x < gl_xl || coord_y < gl_yl  ||
        coord_x >= gl_xu || coord_y >= gl_yu )
        return;

    /* 用障碍物空间点坐标找到其所对应的栅格 */
    int idx_x = static_cast<int>((coord_x - gl_xl) * m_inv_resolution);
    int idx_y = static_cast<int>((coord_y - gl_yl) * m_inv_resolution);

    /* 将障碍物映射为容器里值为1的元素 */
    m_global_obstacle_map[idx_x * GLY_SIZE + idx_y] = 1;
}

void BaseMap::localSetObs(const double coord_x, const double coord_y, const double coord_z)
{
    if (coord_x < gl_xl || coord_y < gl_yl || coord_z < gl_zl ||
        coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu)
        return;
    // ROS_WARN("gl_zl %f", gl_zl);
    /* 用障碍物空间点坐标找到其所对应的栅格 */
    int idx_x = static_cast<int>((coord_x - gl_xl) * m_inv_resolution);
    int idx_y = static_cast<int>((coord_y - gl_yl) * m_inv_resolution);
    int idx_z = static_cast<int>((0 - gl_zl) * m_inv_resolution);
    /* 将障碍物映射为容器里值为1的元素,根据这里的值做raycast process */
    m_local_obstacle_map[idx_x * GLY_SIZE + idx_y] = 1;
}

void BaseMap::getObsEdge(Eigen::Vector2d xcur)
{
    Eigen::Vector3d start_pt;
    start_pt(0) = xcur(0);
    start_pt(1) = xcur(1);
    start_pt(2) = 0.0;
    Eigen::Vector3i start_idx = coord2gridIndex(start_pt);
    start_pt = gridIndex2coord(start_idx);//返回的是中心的坐标

    double start_x = start_pt(0);
    double start_y = start_pt(1);
    double start_z = 0.0;
    double R = 4.0;

    for(int i = -8; i <= 8; i++) /// 这里搜索范围太小会导致一系列的问题，包括但不限于小收敛误差需求时导致的撞墙，
        /// 因为你搜不到较远处的障碍物，但是太多了就会严重拖慢速度。
    {
        for (int j = -8; j <=8; j ++)
        {
            int x_idx = std::max(start_idx.x() + i, 0);
            int y_idx = std::max(start_idx.y() + j, 0);
            int z_idx = start_idx.z();

            Eigen::Vector3i temp_idx = {x_idx, y_idx, z_idx};
            //有占据则说明此地为障碍物
            if (!isFree(x_idx, y_idx, z_idx)) {
                Eigen::Vector3d obs_pt = gridIndex2coord(temp_idx);
                obs_coord.push_back(obs_pt);
            }
        }
    }
    //搜索远处的障碍物
//    std::cout<<"start find far point"<<std::endl;
    for(int i = 0; i < 100; i++){   /// 搜索最近2.5m范围内的点
        double edge_x = start_x + sin(i * M_PI / 50) * R;
        double edge_y = start_y + cos(i * M_PI / 50) * R;
        double edge_z = start_z;

        Eigen::Vector3d obs_pt;
        if(getObsPosition(start_x, start_y, start_z, edge_x, edge_y, edge_z, obs_pt))
        {
            if((obs_pt - start_pt).norm() > 1.0){
                obs_coord.push_back(obs_pt);
            }
        }
    }
}


bool BaseMap::getObsPosition(double start_x, double start_y, double start_z,
                    double edge_x, double edge_y, double edge_z, Eigen::Vector3d &obs_pt)
{
    double path_len = std::sqrt((edge_x - start_x) * (edge_x - start_x) + (edge_y - start_y) * (edge_y - start_y));
    int n = static_cast<int>(path_len * m_inv_resolution * 2) + 1;
    double delta_x = (edge_x - start_x) / n;
    double delta_y = (edge_y - start_y) / n;
    int x_idx, y_idx, z_idx;

    for(int i = 0; i < n; i++){
        double temp_x = start_x + delta_x * i;
        double temp_y = start_y + delta_y * i;
        double temp_z = start_z;
        coord2gridIndex(temp_x, temp_y, temp_z, x_idx, y_idx, z_idx);
        if (!isFree(x_idx, y_idx, z_idx)) {
            obs_pt = {temp_x, temp_y, temp_z};
            return true;
        }
    }
    return false;

}