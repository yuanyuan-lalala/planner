#include"Astar_searcher.hpp"

void AstarPathFinder::visLocalGridMap(const pcl::PointCloud <pcl::PointXYZ> &cloud, const bool swell_flag){
    pcl::PointCloud<pcl::PointXYZ> cloud_vis;
    pcl::PointXYZ pt;
    if(!swell_flag){
        for(auto point : cloud.points){
            pt = point;
            Eigen::Vector3d cor_round = coordRounding(Eigen::Vector3d(pt.x, pt.y, pt.z));
            pt.x = cor_round(0);
            pt.y = cor_round(1);
            pt.z = cor_round(2);
            cloud_vis.points.push_back(pt);
        }

    }else{
        for (int i = 0; i < GLX_SIZE; i++) {
            for (int j = 0; j < GLY_SIZE; j++) {
                for (int k = 0; k < GLZ_SIZE; k++) {
                    Eigen::Vector3i temp_grid(i, j, k);
                    if (isLocalOccupied(temp_grid)) {
                        Eigen::Vector3d temp_pt = gridIndex2coord(temp_grid);
                        pt.x = temp_pt(0);
                        pt.y = temp_pt(1);
                        pt.z = global_map->getHeight(i, j) * 1;
                        cloud_vis.points.push_back(pt);
                    }
                    if(global_map->GridNodeMap[i][j]->exist_second_height_ == true && global_map->GridNodeMap[i][j]->second_local_occupancy_ == true)
                    {
                        Eigen::Vector3d temp_pt = gridIndex2coord(temp_grid);
                        pt.x = temp_pt(0);
                        pt.y = temp_pt(1);
                        pt.z = 2.0;
                        cloud_vis.points.push_back(pt);
                    }
                }
            }
        }

    }

    cloud_vis.width = cloud_vis.points.size();
    cloud_vis.height = 1;
    cloud_vis.is_dense = true;
    sensor_msgs::PointCloud2 map_vis;
    pcl::toROSMsg(cloud_vis, map_vis);
    map_vis.header.frame_id = "world";

    local_grid_map_vis_pub.publish(map_vis);
    // pcl::toROSMsg(cloud_vis, map_vis);
    // map_vis.header.frame_id = "world";

}
/**
 * @brief	用点云坐标生成栅格中心点坐标（相当于降采样）后发给rviz以显示出来
 *          （二维规划下只取指定高度范围的点云）
 * @param
 */
void AstarPathFinder::visGridMap() {
//    ROS_WARN("start publish GridMap");
    pcl::PointCloud <pcl::PointXYZRGB> cloud_vis;
    sensor_msgs::PointCloud2 map_vis;
    pcl::PointXYZRGB pt;

    for (int i = 0; i < GLX_SIZE; i++) {
        for (int j = 0; j < GLY_SIZE; j++) {
            for (int k = 0; k < GLZ_SIZE; k++) {
                Eigen::Vector3i temp_grid(i, j, 0);

//                if(global_map->GridNodeMap[i][j]->exist_second_height == true ){
//                    Eigen::Vector3d temp_pt = gridIndex2coord(temp_grid);
//                    pt.x = temp_pt(0);
//                    pt.y = temp_pt(1);
//                    pt.z = 1.0;
//                    cloud_vis.points.push_back(pt);
//                }
                //检查是否占用，若占用则
                if (global_map->isOccupied(temp_grid, false))
                {   
                    //转化为实际坐标，黑色
                    Eigen::Vector3d temp_pt = gridIndex2coord(temp_grid);
                    pt.x = temp_pt(0);
                    pt.y = temp_pt(1);
                    pt.z = 0.0;
                    pt.r = 255.0;
                    pt.g = 255.0;
                    pt.b = 255.0;
                    cloud_vis.points.push_back(pt);

                }
                    // if (Global::astar_path_finder->getHeight(i, j, k) > -0.99)
                //可见则根据可见度越高，蓝色分量越大
                if(global_map->GridNodeMap[i][j]->visibility_ > 0.1)
                {
                    Eigen::Vector3d temp_pt = gridIndex2coord(temp_grid);
                    pt.x = temp_pt(0);
                    pt.y = temp_pt(1);
                    pt.z = 0.0;
                    pt.r = 255 - global_map->GridNodeMap[i][j]->visibility_ * 12.0;
                    pt.g = 0.0;
                    pt.b = global_map->GridNodeMap[i][j]->visibility_ * 12.0;
                    cloud_vis.points.push_back(pt);
                }

//                // 检查陡坡
//                if(!global_map->isOccupied(temp_grid, false))
//                {
//                    double max_height = -10;
//                    double min_height = 10;
//                    for(int idx = -3; idx <= 3; idx++){
//                        for(int idy = -3; idy <= 3; idy++){
//                            Eigen::Vector3i temp_neigh(i + idx, j + idy, 0);
//                            if (!global_map->isOccupied(temp_neigh, false) && i + idx >= 0 && i + idx < GLX_SIZE && j + idy >= 0 && j + idy < GLY_SIZE){
//                                if(max_height <global_map->GridNodeMap[i + idx][j + idy]->height){
//                                    max_height = global_map->GridNodeMap[i + idx][j + idy]->height;
//                                }
//
//                                if(min_height > global_map->GridNodeMap[i + idx][j + idy]->height){
//                                    min_height = global_map->GridNodeMap[i + idx][j + idy]->height;
//                                }
//                            }
//                        }
//                    }
//                    if(abs(global_map->GridNodeMap[i][j]->height - max_height) > 0.14 || abs(global_map->GridNodeMap[i][j]->height - min_height) > 0.14){
//                        Eigen::Vector3d temp_pt = gridIndex2coord(temp_grid);
//                        pt.x = temp_pt(0);
//                        pt.y = temp_pt(1);
//                        pt.z = 0.5;
//                        cloud_vis.points.push_back(pt);
//                    }
//                }
//                  //// 检查下台阶
//                if(!global_map->isOccupied(temp_grid, false))
//                {
//                    double max_height = -10;
//                    double min_height = 10;
//                    for(int idx = -1; idx <= 1; idx++){
//                        for(int idy = -1; idy <= 1; idy++){
//                            Eigen::Vector3i temp_neigh(i + idx, j + idy, 0);
//                            if (!global_map->isOccupied(temp_neigh, false) && i + idx >= 0 && i + idx < GLX_SIZE && j + idy >= 0 && j + idy < GLY_SIZE){
//                                if(max_height <global_map->GridNodeMap[i + idx][j + idy]->height){
//                                    max_height = global_map->GridNodeMap[i + idx][j + idy]->height;
//                                }
//
//                                if(min_height > global_map->GridNodeMap[i + idx][j + idy]->height){
//                                    min_height = global_map->GridNodeMap[i + idx][j + idy]->height;
//                                }
//                            }
//                        }
//                    }
//                    if(abs(global_map->GridNodeMap[i][j]->height - max_height) > 0.12 || abs(global_map->GridNodeMap[i][j]->height - min_height) > 0.12){
//                        Eigen::Vector3d temp_pt = gridIndex2coord(temp_grid);
//                        pt.x = temp_pt(0);
//                        pt.y = temp_pt(1);
//                        pt.z = 0.5;
//                        cloud_vis.points.push_back(pt);
//                    }
//                }
                  // 检查桥洞

            }
        }
    }

    /* 设置点云的规则（无序、坐标值有限） */
//    cloud_vis.width = cloud_vis.points.size();
//    cloud_vis.height = 1;
//    cloud_vis.is_dense = true;

    /* 将障碍物的PCL点云数据类型转换为ROS点云通信数据类型*/
    pcl::toROSMsg(cloud_vis, map_vis);

    /* 设置坐标系的名称 */
    map_vis.header.frame_id = "world";

    /* 发布障碍物点云到rviz中 */
    grid_map_vis_pub.publish(map_vis);
    int temp = 1;
    while (temp--) {
        pcl::toROSMsg(cloud_vis, map_vis);
        map_vis.header.frame_id = "world";
        grid_map_vis_pub.publish(map_vis);
    }
}


Eigen::Vector3d AstarPathFinder::coordRounding(const Eigen::Vector3d &coord)
{
    return gridIndex2coord(coord2gridIndex(coord));
}

Eigen::Vector3i AstarPathFinder::coord2gridIndex(const Eigen::Vector3d &pt)
{
    Eigen::Vector3i idx;
    idx << std::min(std::max(int((pt(0) - gl_xl) * m_inv_resolution), 0), GLX_SIZE - 1),
        std::min(std::max(int((pt(1) - gl_yl) * m_inv_resolution), 0), GLY_SIZE - 1),
        std::min(std::max(int((pt(2) - gl_zl) * m_inv_resolution), 0), GLZ_SIZE - 1);

    return idx;
}

Eigen::Vector3d AstarPathFinder::gridIndex2coord(const Eigen::Vector3i &index)
{
    Eigen::Vector3d pt;

    pt(0) = ((double)index(0) + 0.5) * m_resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * m_resolution + gl_yl;
    pt(2) = ((double)index(2) + 0.5) * m_resolution + gl_zl;

    return pt;
}

bool AstarPathFinder::isLocalOccupied(const Eigen::Vector3i &index) const
{
    return isLocalOccupied(index(0), index(1), index(2));
}

bool AstarPathFinder::isLocalOccupied(const int &idx_x, const int &idx_y, const int &idx_z) const
{
    //进行二维平面的占据
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE &&
            (global_map->l_data[idx_x * GLY_SIZE + idx_y] == 1));
//    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE &&
//            (global_map->OccupancyMap[idx_x][idx_y]->occ == 1));
}

bool AstarPathFinder::isOccupied(const Eigen::Vector3i &index) const
{
    return isOccupied(index(0), index(1), index(2));
}

bool AstarPathFinder::isOccupied(const int &idx_x, const int &idx_y, const int &idx_z) const
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE &&
            (global_map->data[idx_x * GLY_SIZE + idx_y] == 1 || global_map->l_data[idx_x * GLY_SIZE + idx_y] == 1));
//    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE &&
//            (global_map->data[idx_x * GLY_SIZE + idx_y] == 1 || (global_map->OccupancyMap[idx_x][idx_y]->occ == 1)));
}

/**
 * @brief	路径节点二次规划（筛除冗余节点、平滑处理）

 * @param
 */
std::vector<Eigen::Vector3d> AstarPathFinder::smoothPath(std::vector<Eigen::Vector3d> src_path, bool continuous = true)
{
    if (src_path.size() < 2)
    {
        return src_path;
    }
    /* 定义一个容器用于存放优化后的路径节点 */
    std::vector<Eigen::Vector3d> optimized_path;

    /* 取出第一个路径节点作为第一个尾节点 */
    Eigen::Vector3d tailPos = src_path.back();
    src_path.pop_back();

    optimized_path.push_back(tailPos);

    /* 取出第二个路径节点作为第一个中间节点 */
    Eigen::Vector3d midPos = src_path.back();
    //    src_path.pop_back();  /// TODO

    /* 定义首节点 */
    Eigen::Vector3d headPos;
    int count = 0;

    while (!src_path.empty())
    {
        /* 首节点往前延伸 */
        headPos = src_path.back();

        /* 计算在首节点和尾节点之间的连线上分段 */
        double len = pow((pow(headPos.x() - tailPos.x(), 2) +
                          pow(headPos.y() - tailPos.y(), 2)),
                         0.5);
        int n = static_cast<int>(len / (0.2795085 * m_resolution)) + 1;
        double deta_x = (headPos.x() - tailPos.x()) / n;
        double deta_y = (headPos.y() - tailPos.y()) / n;

        /* 如果连线上存在某个点落在障碍物栅格中则认为连线经过障碍物 */
        for (int i = 1; i < n; i++)
        {
            double temp_x = tailPos.x() + deta_x * i;
            double temp_y = tailPos.y() + deta_y * i;
            double temp_z = 0;
            Eigen::Vector3d tempPos = {temp_x, temp_y, temp_z};

            /* 用空间点坐标找到其所对应的栅格 */
            Eigen::Vector3i tempID = coord2gridIndex(tempPos);

            /* 如果两点连线经过障碍物栅格，则把中间点加入优化路径中去 */
            if (isOccupied(tempID))
            {
                if (!continuous)
                {
                    Eigen::Vector3d collision_pos;
                    std::vector<Eigen::Vector3d> temppath = {midPos, tailPos};
                    if (checkPathCollision(temppath, collision_pos))
                    {
                        count ++ ;
                        ROS_WARN("occupied!: %f %f", tempPos.x(), tempPos.y());
                        //找到中间的点，如果有障碍物，找能绕开障碍物的点
                        Eigen::Vector3d Pos = getMidPoint(temppath);
                        optimized_path.push_back(Pos);
                        src_path.push_back(Pos);
                        ROS_WARN("occupied Pos!: %f %f", Pos.x(), Pos.y());
                        midPos = Pos;
                        ROS_WARN("smooth check collision : %d", count);
                    }
                }
                optimized_path.push_back(midPos);
                tailPos = midPos;
                break;
            }
        }
        midPos = headPos;
        src_path.pop_back();
        if(count > 1000)
        {
            break;
        }

    }
    /* 添加最后一个路径节点(目标节点) */
    optimized_path.push_back(midPos);

//    ROS_WARN("Optimized path has %d node", (int)optimized_path.size());
    return optimized_path;
}

/**
 * @brief	检查路径是否与障碍物碰撞
*/
bool AstarPathFinder::checkPathCollision(std::vector<Eigen::Vector3d> optimized_path, Eigen::Vector3d &collision_pos)
{

    /* 取出第一个路径节点作为第一个尾节点，此后tailPos记录的就一直是检查碰撞的上一个节点 */
    int optimized_path_size = optimized_path.size();
    Eigen::Vector3d tailPos = optimized_path.back();
    optimized_path.pop_back();

    /* 定义首节点 */
    Eigen::Vector3d headPos;

    while (!optimized_path.empty()) {
        headPos = optimized_path.back();
        optimized_path.pop_back();
        double path_len = sqrt(pow((tailPos.x() - headPos.x()), 2) + pow((tailPos.y() - headPos.y()), 2));
        // 1/根号5
        int n = static_cast<int>(path_len / (0.2795085 * m_resolution)) + 1;
        double deta_x = (headPos.x() - tailPos.x()) / n;
        double deta_y = (headPos.y() - tailPos.y()) / n;
        /* 如果连线上存在某个点落在障碍物栅格中则认为连线经过障碍物 */
        for (int i = 1; i < n; i++) {
            double temp_x = tailPos.x() + deta_x * i;
            double temp_y = tailPos.y() + deta_y * i;
            double temp_z = 0;
            Eigen::Vector3d tempPos = {temp_x, temp_y, temp_z};
            /* 用空间点坐标找到其所对应的栅格 */
            Eigen::Vector3i tempID = coord2gridIndex(tempPos);

            /* 如果两点连线经过障碍物栅格，则把返回true */
            if (isOccupied(tempID)) {
                collision_pos = tempPos;
                ROS_WARN("collision_pos!: %f %f", collision_pos.x(), collision_pos.y());
                return true;
            }
        }
        tailPos = headPos;
    }
    return false;
}

std::vector<Eigen::Vector3d> AstarPathFinder::smoothTopoPath(std::vector<Eigen::Vector3d> topo_path)
{
    if(topo_path.size() < 2)
    {
        return topo_path;
    }
    Eigen::Vector3d tail_pos, colli_pt;
    std::vector<Eigen::Vector3d> smooth_path;
    Eigen::Vector3d head_pos = topo_path[0];
    smooth_path.push_back(head_pos);
    int iter_idx = 0;
    bool collision = true;
    double last_height = head_pos.z();  // 初始化高度为起点高度，防止起点在桥洞区域
    int iter_count = 0;

    while(collision)  // 如果迭代到最后一个点不出现碰撞，则认为可以直接把当前点连接到终点，整体优化完毕
    {
        iter_count++;
        if(iter_count > 1000){
            ROS_WARN("[A Star ERROR] -------- generated trajectory is not safe! --------");
            break;
        }
        collision = false;
        Eigen::Vector3d temp;
        for(int i = iter_idx; i < topo_path.size() - 1; i++)  // 找topo路径的每一段
        {
            last_height = topo_path[i].z();

            double x_offset = topo_path[i + 1].x() - topo_path[i].x();
            double y_offset = topo_path[i + 1].y() - topo_path[i].y();
            double distance = std::sqrt(pow(x_offset, 2) + pow(y_offset, 2));

            int n = std::sqrt(pow(x_offset, 2) + pow(y_offset, 2)) / 0.05;  // 在每一段采样
            for (int j = 1; j <= n; j++)
            {
                bool second_height = false;
                //采样间隔为0.05m
                tail_pos.x() = topo_path[i].x() + j * 0.05 * x_offset / distance;
                tail_pos.y() = topo_path[i].y() + j * 0.05 * y_offset / distance;
                Eigen::Vector3i temp_id = coord2gridIndex(tail_pos);
                //正常求z
                if(!global_map->GridNodeMap[temp_id.x()][temp_id.y()]->exist_second_height_){
                    tail_pos.z() = global_map->GridNodeMap[temp_id.x()][temp_id.y()]->height_;
                    last_height = tail_pos.z();
                }else{//有桥洞
                    tail_pos.z() = last_height;  //进入桥洞区域后高度等于last height
                    if(tail_pos.z() < 0.4){
                        second_height = true;  // 在桥洞区域我们要判断这个点是不是在第二高度上
                    }
                }
                //查看地图占用，如果占用先不处理
                if(global_map->isOccupied(temp_id.x(), temp_id.y(), temp_id.z(), second_height)){
                    continue;  // TODO 暂时处理为不考虑这种情况
                }

                if (!lineVisib(tail_pos, head_pos, colli_pt, 0.05) && !collision){  // 记录最近的可见点的第一个不可见点，这样如果中间又有可见的点是就可以把collision改成true
                    collision = true;
                    Eigen::Vector3i temp_id = coord2gridIndex(tail_pos);
                    temp = {topo_path[i].x() + (j) * 0.05 * x_offset / distance,
                            topo_path[i].y() + (j) * 0.05 * y_offset / distance,
                            last_height};
                    temp = tail_pos;
                    bool easy = getNearPoint(temp, head_pos, temp);
                    iter_idx = i;
                }
                else if (lineVisib(tail_pos, head_pos, colli_pt, 0.05)){
                    // 这里的意思是，只要最后一段可见，就不用再检查了，直接连。即为连接最后一个可见的topo点
                    collision = false;
                }
            }
        }
        if(collision){
            head_pos = temp;
//            last_height = head_pos.z();
            smooth_path.push_back(temp);
        }else{
            smooth_path.push_back(topo_path.back());
            break;
        }
    }
    if(iter_count>1000){
        return topo_path;
    }
    return smooth_path;
}

Eigen::Vector3d AstarPathFinder::getMidPoint(std::vector<Eigen::Vector3d> midpath)
{
    /* 取出第一个路径节点作为第一个尾节点 */
    Eigen::Vector3d tailPos = midpath.back();
    midpath.pop_back();

    /* 定义首节点 */
    Eigen::Vector3d headPos;
    Eigen::Vector3d nearPos;

    while (!midpath.empty())
    {
        headPos = midpath.back();
        midpath.pop_back();
        double path_len = sqrt(pow((tailPos.x() - headPos.x()), 2) + pow((tailPos.y() - headPos.y()), 2));
        int n = static_cast<int>(path_len / (0.2795085 * m_resolution)) + 1;
        double deta_x = (headPos.x() - tailPos.x()) / n;
        double deta_y = (headPos.y() - tailPos.y()) / n;
        /* 如果连线上存在某个点落在障碍物栅格中则认为连线经过障碍物 */
        for (int i = 1; i < n; i++)
        {
            double temp_x = tailPos.x() + deta_x * i;
            double temp_y = tailPos.y() + deta_y * i;
            double temp_z = 0;
            Eigen::Vector3d tempPos = {temp_x, temp_y, temp_z};

            /* 用空间点坐标找到其所对应的栅格 */
            Eigen::Vector3i tempID = coord2gridIndex(tempPos);

            /* 如果两点连线经过障碍物栅格，则处理返回新的点 */
            if (isOccupied(tempID))
            {
                if(getNearPoint(headPos, tailPos, nearPos))
                {
                    return nearPos;
                }
                else
                {
                    return headPos;
                }
            }
        }
    }
    return headPos;
}

bool AstarPathFinder::lineVisib(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, Eigen::Vector3d& colli_pt, double thresh)
{
    double ray_ptx, ray_pty, ray_ptz;
    double p2_x, p2_y, p2_z, p1_x, p1_y, p1_z;
    double distance_thresh;
    p2_x = p2.x();
    p2_y = p2.y();
    p2_z = p2.z();

    p1_x = p1.x();
    p1_y = p1.y();
    p1_z = p1.z();

    double x_offset = p1.x() - p2.x();
    double y_offset = p1.y() - p2.y();
    double z_offset = p1.z() - p2.z();

    double distance = std::sqrt(pow(x_offset, 2) + pow(y_offset, 2));

    int n = int(double(std::sqrt(pow(x_offset, 2) + pow(y_offset, 2))) / double(thresh));
    int idx, idy, idz, idx_end, idy_end, idz_end;
    global_map->coord2gridIndex(p2_x, p2_y, p2_z, idx, idy, idz);
    global_map->coord2gridIndex(p1_x, p1_y, p1_z, idx_end, idy_end, idz_end);
    double last_height = p2_z;  ///起点高度

    for(int i = 0; i < n + 1; i++)
    {
        if(i == n){
            ray_ptx = p1_x;
            ray_pty = p1_y;
            ray_ptz = p1_z;
        }else {
            ray_ptx = p2_x + i * thresh * x_offset / distance;
            ray_pty = p2_y + i * thresh * y_offset / distance;
            ray_ptz = 0.0;
        }

        int pt_idx, pt_idy, pt_idz;
        global_map->coord2gridIndex(ray_ptx, ray_pty, ray_ptz, pt_idx, pt_idy, pt_idz);
        double height;
        bool second_height = false;
        if(!global_map->GridNodeMap[pt_idx][pt_idy]->exist_second_height_) {
            height = global_map->getHeight(pt_idx, pt_idy);  //非桥洞区域正常通行
        }else{
            height = last_height;
            if(global_map->GridNodeMap[idx_end][idy_end]->exist_second_height_){
                last_height = p1_z;
            }
            if(height < 0.4){
                //如果当前栅格存在第二高度（如桥洞），则使用上一个采样点的高度，避免在高度变化较大的区域产生错误的判断。
                second_height = true;  // 在桥洞区域我们要判断这个点是不是在第二高度上
            }
        }
        if (global_map->isOccupied(pt_idx, pt_idy, pt_idz, second_height)){
            Eigen::Vector3i temp_idx = {pt_idx, pt_idy, pt_idz};
            colli_pt = gridIndex2coord(temp_idx);
            return false;
        }
        else if (height - last_height >= 0.12)//高度变化过大
        {
            Eigen::Vector3i temp_idx = {pt_idx, pt_idy, pt_idz};
            colli_pt = gridIndex2coord(temp_idx);
            return false;
        }
        else if (height - last_height <= -0.3)//路径不可通行
        {
            Eigen::Vector3i temp_idx = {pt_idx, pt_idy, pt_idz};
            colli_pt = gridIndex2coord(temp_idx);
            return false;
        }
        last_height = height;
    }
    return true;
}
// 存在障碍物的情况下。通过对 headPos 进行偏移，函数试图找到一个安全的位置，返回该位置作为最佳点。
bool AstarPathFinder::getNearPoint(Eigen::Vector3d headPos, Eigen::Vector3d tailPos, Eigen::Vector3d &best_point)
{
    double check_distance = 0.3;
    while(check_distance >= 0.1)
    {
        double delta_x = headPos.x() - tailPos.x();
        double delta_y = headPos.y() - tailPos.y();
        Eigen::Vector3d collision_pt;

        Eigen::Vector3d headPos_left, headPos_right;
        headPos_left.x() = headPos.x() + check_distance * delta_y / (sqrt(pow(delta_y, 2) + pow(delta_x, 2)));
        headPos_left.y() = headPos.y() - check_distance * delta_x / (sqrt(pow(delta_y, 2) + pow(delta_x, 2)));
        headPos_left.z() = headPos.z();

        headPos_right.x() = headPos.x() - check_distance * delta_y / (sqrt(pow(delta_y, 2) + pow(delta_x, 2)));
        headPos_right.y() = headPos.y() + check_distance * delta_x / (sqrt(pow(delta_y, 2) + pow(delta_x, 2)));
        headPos_right.z() = headPos.z();

        Eigen::Vector3i tempID_left = coord2gridIndex(headPos_left);
        Eigen::Vector3i tempID_right = coord2gridIndex(headPos_right);
        //检查膨胀距离
        int check_swell = (check_distance / 0.1) -1;
        if (lineVisib(tailPos, headPos_left, collision_pt, 0.1))
        {
            best_point = headPos_left;
            return true;
        }else if (lineVisib(tailPos, headPos_right, collision_pt, 0.1)){
            best_point = headPos_right;
            return true;
        }else{
            best_point = headPos;
        }

        check_distance -= 0.1;
    }

    return false;
}

bool AstarPathFinder::findNeighPoint(Eigen::Vector3i path_point, Eigen::Vector3i &output_point, int check_num)
{
    Eigen::Vector3i path_succ;
    std::vector<double> path_distance;
    std::vector<Eigen::Vector3i> path_free;
    for (int idx = -check_num; idx <= check_num; idx++)
    {
        for (int jdx = -check_num; jdx <= check_num; jdx++)
        {
            path_succ.x() = path_point.x() + idx;
            path_succ.y() = path_point.y() + jdx;
            path_succ.z() = path_point.z();
            //检查是不是障碍物或者已经被访问过
            if (isFree(path_succ))
            {
                path_free.push_back(path_succ);
                double distance = std::sqrt(std::pow(path_succ.x() - path_point.x(), 2) + std::pow(path_succ.y() - path_point.y(), 2));
                path_distance.push_back(distance);
                // output_point = path_succ;
                // return true;
            }
        }
    }
    if (path_free.empty())
    {
        return false;
    }
    std::vector<double>::iterator maxdistance = std::max_element(path_distance.begin(), path_distance.end());
    int index = std::distance(path_distance.begin(), maxdistance);
    output_point = path_free[index];
    return true;
}

bool AstarPathFinder::isFree(const Eigen::Vector3i &index) const
{
    return isFree(index(0), index(1), index(2));
}

bool AstarPathFinder::isFree(const int &idx_x, const int &idx_y, const int &idx_z) const
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE &&
            (global_map->data[idx_x * GLY_SIZE + idx_y] < 1 && global_map->l_data[idx_x * GLY_SIZE + idx_y] < 1));
//    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE &&
//            (global_map->data[idx_x * GLY_SIZE + idx_y] < 1 && global_map->OccupancyMap[idx_x][idx_y]->occ < 1));
}