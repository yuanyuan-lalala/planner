#include "global_map.hpp"

double GlobalMap::getHeight(const int idx_x, const int idx_y)
{
     if (idx_x > GLX_SIZE || idx_y > GLY_SIZE)
         return 0.0;
     return GridNodeMap[idx_x][idx_y]->height_;
}


void GlobalMap::initGridMap(ros::NodeHandle &nh, std::string occ_map_file, std::string bev_map_file, std::string distance_map_file,
                            double resolution, Eigen::Vector3d global_xyz_l, Eigen::Vector3d global_xyz_u,
                            int max_x_id, int max_y_id, int max_z_id, double robot_radius,
                            double _2d_search_height_low, double _2d_search_height_high, double search_radius)
{
    m_nh = nh;

    /* 设置地图的边界与尺寸 */
    gl_xl = global_xyz_l(0);
    gl_yl = global_xyz_l(1);
    gl_zl = global_xyz_l(2);
    gl_xu = global_xyz_u(0);
    gl_yu = global_xyz_u(1);
    gl_zu = global_xyz_u(2);

    nh.param("trajectory_generator/height_bias", m_height_bias, -0.5);
    nh.param("trajectory_generator/height_interval", m_height_interval, 2.0);
    nh.param("trajectory_generator/height_threshold", m_height_threshold, 0.1);
    nh.param("trajectory_generator/height_sencond_high_threshold", m_height_sencond_high_threshold, 0.4);

    std::cout<<"m_height_bias: "<<m_height_bias<<" m_height_interval: "<<m_height_interval
             <<" m_height_threshold: "<<m_height_threshold<<" height_sencond_high_threshold: "<<m_height_sencond_high_threshold<<std::endl;
    
    GLX_SIZE = max_x_id;
    GLY_SIZE = max_y_id;
    GLZ_SIZE = max_z_id;
    GLXY_SIZE = GLX_SIZE * GLY_SIZE;
    GLYZ_SIZE = GLY_SIZE * GLZ_SIZE;
    GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;
    m_resolution = resolution;
    m_inv_resolution = 1.0 / resolution;
    //二维平面
    data = new uint8_t[GLXY_SIZE];
    l_data = new uint8_t[GLXY_SIZE];
    memset(data, 0, GLXY_SIZE * sizeof(uint8_t));
    memset(l_data, 0, GLXY_SIZE * sizeof(uint8_t));
    //设置2D搜索高度 
    m_2d_search_height_low = _2d_search_height_low;
    m_2d_search_height_high = _2d_search_height_high;
    /* 设置机器人碰撞检测半径 */
    m_robot_radius = robot_radius;
    m_robot_radius_dash = robot_radius;
    m_search_radius = search_radius;
    //不用考虑颜色，只需要查看是否占据即可
    cv::Mat occ_map3c, bev_map,occ_map1c,distance_map,topo_map;
    
    occ_map3c = cv::imread(occ_map_file);
    bev_map = cv::imread(bev_map_file);
    distance_map = cv::imread(distance_map_file);
    
    std::vector <cv::Mat> channels;
    cv::split(occ_map3c, channels);
    occ_map1c = channels.at(0);
    
    std::vector <cv::Mat> topo_channels;
    cv::split(distance_map, topo_channels);
    topo_map = topo_channels.at(0);

    // 四个回调函数分别对应真实与仿真环境的回调数据
    //用雷达数据进行局部地图构建
    m_local_point_sub = m_nh.subscribe<sensor_msgs::PointCloud2>("/cloud_registered_world", 1, &GlobalMap::lidarCloudCallback, this);
    m_realsense_sub =  m_nh.subscribe<sensor_msgs::PointCloud2>("/realsense_pointcloud", 1, &GlobalMap::realsenseCloudCallback, this);
    m_odometry_sub = m_nh.subscribe<nav_msgs::Odometry>("/odometry_imu", 1, &GlobalMap::odometryCallback, this);

    gazebo_point_sub = m_nh.subscribe<sensor_msgs::PointCloud2>("/mbot/velodyne_points", 1, &GlobalMap::gazeboCloudCallback, this);
    gazebo_odometry_sub = m_nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1, &GlobalMap::gazeboPosCallback, this);

    global_map_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/grid_map_vis", 1);
    //    m_occ_timer = m_node.createTimer(ros::Duration(0.051), &GlobalMap::updateLocalMap, this);

    int swell_num = (int) (m_robot_radius * m_inv_resolution) * 2;//膨胀数目
    int GL_SIZE = swell_num + (int)(m_search_radius * m_inv_resolution);

    GridNodeMap = new GridNodePtr *[GLX_SIZE];
    for (int i = 0; i < GLX_SIZE; i++){
        GridNodeMap[i] = new GridNodePtr [GLY_SIZE];
        for (int j = 0; j < GLY_SIZE; j++){
            Eigen::Vector3i tmpIdx(i, j, 0);
            Eigen::Vector3d pos = gridIndex2coord(tmpIdx);
            GridNodeMap[i][j] = new GridNode(tmpIdx, pos);
        }
    }

    GridNodeLocalMap = new GridNodePtr *[GLX_SIZE];
    for (int i = 0; i < GLX_SIZE; i++)
    {
        GridNodeLocalMap[i] = new GridNodePtr[GLY_SIZE];
        for (int j = 0; j < GLY_SIZE; j++)
        {
            Eigen::Vector3i tmpIdx(i, j, 0);
            Eigen::Vector3d pos = gridIndex2coord(tmpIdx);
            GridNodeLocalMap[i][j] = new GridNode(tmpIdx, pos);
        }
    }

    OccupancyMap = new OccupancyNodePtr *[GLX_SIZE];
    for (int i=0;i<GLX_SIZE;i++){
        OccupancyMap[i] = new OccupancyNodePtr [GLY_SIZE];
        for (int j=0;j<GLY_SIZE;j++)
        {
            Eigen::Vector3i tmpIdx(i, j, 0);
            OccupancyMap[i][j] = new OccupancyNode(tmpIdx, false);  // 初始化设为全部已知
        }
    }

    occ_map1c = swellOccMap(occ_map1c);
    bevMapToHeight(bev_map, occ_map1c);
    processSecondHeights();

    occMaptoObstacle(occ_map1c);
    //    pruneGraph(topo_map);
    topoSampleMap(topo_map);

    m_local_cloud.reset(new pcl::PointCloud <pcl::PointXYZ>);
}


cv::Mat GlobalMap::swellOccMap(cv::Mat occ_map1c)
{
    cv::Mat occ = cv::Mat::zeros(occ_map1c.rows, occ_map1c.cols, CV_8UC1);
    int swell_num = (int) (m_robot_radius * m_inv_resolution);//几格
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


void GlobalMap::bevMapToHeight(const cv::Mat bev_map, const cv::Mat occ_map1c)
{
    /// 高度地图建立！
    std::vector<cv::Mat> channels;
    cv::split(bev_map, channels);
    cv::Mat bev = channels.at(0);

    std::vector<cv::Mat> channels_occ;
    cv::split(occ_map1c, channels_occ);
    cv::Mat occ = channels_occ.at(0);

    pcl::PointXYZ pt;
    for (int i = 1; i < bev.rows -1 ; i++){
        for (int j = 1; j < bev.cols - 1; j++){
            bool exist_heights = false;;
            pt.x = (j + 0.5) * m_resolution;
            pt.y = (bev.rows - i - 0.5) * m_resolution;
            pt.z = (bev.at<uchar>(i, j) * m_height_interval / 255) + m_height_bias; /// TODO map param

            if (occ.at<uchar>(i, j) == 0) {
                for (int idx = -1; idx <= 1; idx++) {
                    for (int idy = -1; idy <= 1; idy++) {
                        //将bev的单通道灰度转化为高度
                        double temp_z = (bev.at<uchar>(i+idx, j+idy) * m_height_interval / 255) + m_height_bias;
                        if(abs(temp_z - pt.z) > 0.4){
                            exist_heights = true;
                            second_heights_points.push_back(Eigen::Vector3d(pt.x, pt.y, pt.z));
                        }
                    }
                }
            }
            setHeight(pt.x, pt.y, pt.z, exist_heights);  // 如果高度变化太大的可通行区域点视为桥洞出入口
        }
    }
     std::cout<<"second_heights_points: "<<second_heights_points.size()<<std::endl;
    ROS_WARN("[Map Build] bev build done");
}

void GlobalMap::processSecondHeights()
{   // 处理桥洞的高度
    while(!second_heights_points.empty()){
        Eigen::Vector3d temp_point = second_heights_points.back();
        double x_max = temp_point.x();
        double x_min = temp_point.x();
        double y_max = temp_point.y();
        double y_min = temp_point.y();
        second_heights_points.pop_back();
        for(std::vector<Eigen::Vector3d>::iterator it = second_heights_points.begin(); it != second_heights_points.end(); it++){
            if(std::sqrt((temp_point.x() - (*it).x()) * (temp_point.x() - (*it).x())
                        + (temp_point.y() - (*it).y()) * (temp_point.y() - (*it).y())) < 4.0){
                x_max = std::max(x_max, (*it).x());
                x_min = std::min(x_min, (*it).x());
                y_max = std::max(y_max, (*it).y());
                y_min = std::min(y_min, (*it).y());
                it = second_heights_points.erase(it);
                it--;
            }
        }
        std::cout<<"xmax: "<<x_max<<" xmin: "<<x_min<<" ymax: "<<y_max<<" ymin: "<<y_min<<std::endl;
        int idx_x_max = static_cast<int>((x_max - gl_xl + 0.01) * m_inv_resolution);
        int idx_y_max = static_cast<int>((y_max - gl_yl + 0.01) * m_inv_resolution);
        int idx_x_min = static_cast<int>((x_min - gl_xl + 0.01) * m_inv_resolution);
        int idx_y_min = static_cast<int>((y_min - gl_yl + 0.01) * m_inv_resolution);
        for(int i = idx_x_min; i<=idx_x_max; i++) {
            for (int j = idx_y_min; j <= idx_y_max; j++) {
                GridNodeMap[i][j]->second_local_swell_=true;
                for(int k = -2; k <= 2; k++){
                    for(int m = -2; m <= 2; m++){
                        GridNodeMap[i + k][j + m]->exist_second_height_ = true;
                    }
                }
            }
        }
    }
}


void GlobalMap::occMaptoObstacle(const cv::Mat occ_map)
{
    pcl::PointXYZ pt;
    ROS_WARN("R.rows %d R.cols %d", occ_map.rows, occ_map.cols);

    for (int i = 0; i < occ_map.rows; i++)
    {
        for (int j = 0; j < occ_map.cols; j++)
        {
            if (occ_map.at<uchar>(i, j) != 0)
            {
                pt.x = (j+0.5) * m_resolution;
                pt.y = (occ_map.rows - i - 0.5) * m_resolution;
                setObs(pt.x, pt.y);
            }
        }
    }
}

void GlobalMap::topoSampleMap(cv::Mat topo_map)
{   // 建立topo采样路径点
    ROS_WARN("start topo sample map");
    Eigen::Vector3d topoMapTemp;
    for(int i = 0; i < topo_map.rows; i++){
        for(int j = 0; j < topo_map.cols; j++){
            if(topo_map.at<uchar>(i, j) > 10){
                topoMapTemp.x() = (j + 0.5) * m_resolution;
                topoMapTemp.y() = (topo_map.rows - i - 0.5) * m_resolution;
                topoMapTemp.z() = 0.0;

                int keypoint = 0;
                //关键点
                for(int idx = -1; idx < 2; idx++){
                    for(int idy = -1; idy < 2; idy++){
                        if(topo_map.at<uchar>(i + idx, j + idy) > 100){
                            keypoint ++ ;
                        }
                    }
                }
                if(keypoint > 3){  // 多路径交汇点作为topo的关键点必进入topo search中
                    int pt_idx, pt_idy, pt_idz;
                    coord2gridIndex(topoMapTemp.x(), topoMapTemp.y(), topoMapTemp.z(), pt_idx, pt_idy, pt_idz);
                    topoMapTemp.z() = getHeight(pt_idx, pt_idy);
                    topo_keypoint.push_back(topoMapTemp);

                    if(GridNodeMap[pt_idx][pt_idy]->exist_second_height_){  // 桥洞区域的所有点双倍采样，分别设置桥上和桥下点
                        topoMapTemp.z() = GridNodeMap[pt_idx][pt_idy]->second_height_;
                        topo_keypoint.push_back(topoMapTemp);
                    }

                    for(int idx = -1; idx < 2; idx++){
                        for(int idy = -1; idy < 2; idy++){
                            topo_map.at<uchar>(i + idx, j + idy) = 0;
                        }
                    }
                }
                if(keypoint == 3){
                    int pt_idx, pt_idy, pt_idz;
                    coord2gridIndex(topoMapTemp.x(), topoMapTemp.y(), topoMapTemp.z(), pt_idx, pt_idy, pt_idz);
                    topoMapTemp.z() = getHeight(pt_idx, pt_idy);
                    topo_sample_map.push_back(topoMapTemp);

                    if(GridNodeMap[pt_idx][pt_idy]->exist_second_height_){  //桥洞同等处理
                        topoMapTemp.z() = GridNodeMap[pt_idx][pt_idy]->second_height_;
                        topo_sample_map.push_back(topoMapTemp);
                    }

                }
            }
        }
    }
    ROS_WARN("topo_sample_map size: %d", topo_sample_map.size());
    ROS_WARN("topo_keypoint size: %d", topo_keypoint.size());
}

void GlobalMap::setHeight(const double coord_x, const double coord_y, const double coord_z, const bool exist_heights)
{
    if (coord_x < gl_xl || coord_y < gl_yl ||
        coord_x >= gl_xu || coord_y >= gl_yu )
        return;

    /* 用障碍物空间点坐标找到其所对应的栅格 */
    int idx_x = static_cast<int>((coord_x - gl_xl + 0.01) * m_inv_resolution);
    int idx_y = static_cast<int>((coord_y - gl_yl + 0.01) * m_inv_resolution);

    GridNodeMap[idx_x][idx_y]->height_ = coord_z;
    GridNodeMap[idx_x][idx_y]->exist_second_height_ = exist_heights;
}

/**
 * @brief	设置障碍物（将障碍物信息保存到路径规划器里）
 *          膨胀处理（防撞设计）
 * @param   组成障碍物空间点坐标
 */
void GlobalMap::setObs(const double coord_x, const double coord_y)
{
    if (coord_x < gl_xl || coord_y < gl_yl  ||
        coord_x >= gl_xu || coord_y >= gl_yu )
        return;

    /* 用障碍物空间点坐标找到其所对应的栅格 */
    int idx_x = static_cast<int>((coord_x - gl_xl) * m_inv_resolution);
    int idx_y = static_cast<int>((coord_y - gl_yl) * m_inv_resolution);

    /* 将障碍物映射为容器里值为1的元素 */
    data[idx_x * GLY_SIZE + idx_y] = 1;
}

void GlobalMap::setRadiusDash(const double dash_radius)
{
    m_robot_radius_dash = dash_radius;
}


bool GlobalMap::isOccupied(const Eigen::Vector3i &index, bool second_height) const
{
    return isOccupied(index(0), index(1), index(2), second_height);
}

bool GlobalMap::isOccupied(const int &idx_x, const int &idx_y, const int &idx_z, bool second_height) const
{  // 桥洞区域判断是否被占用需要特意处理
    if(second_height){
        return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE &&
                (data[idx_x * GLY_SIZE + idx_y] == 1 || GridNodeMap[idx_x][idx_y]->second_local_occupancy_ == true));
    }
    //要么是静态地图，要么是动态地图
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 &&
            (data[idx_x * GLY_SIZE + idx_y] == 1 || l_data[idx_x * GLY_SIZE + idx_y] == 1));
}


void GlobalMap::realsenseCloudCallback(const sensor_msgs::PointCloud2ConstPtr &point){  // 435点膨胀半径小
    
    static pcl::PointCloud<pcl::PointXYZ>::Ptr local_cloud_3d(new pcl::PointCloud<pcl::PointXYZ>);
    local_cloud_3d->clear();
    m_local_cloud->clear();

    pcl::fromROSMsg(*point, *local_cloud_3d);

    /* 点云区域筛选 */
    pcl::PointXYZ pt;
    pcl::PointXYZ pt_3d;
    int point_size = 0;

    for (int i = 0; i < (int)local_cloud_3d->points.size(); i++)
    {
        pt_3d = local_cloud_3d->points[i];
        if (pt_3d.z > m_2d_search_height_low && pt_3d.z < m_2d_search_height_high && (abs(pt_3d.x - odom_position(0)) < search_radius) && (abs(pt_3d.y - odom_position(1)) < search_radius))
        {
            pt.x = pt_3d.x;
            pt.y = pt_3d.y;
            pt.z = pt_3d.z;
            m_local_cloud->points.push_back(pt);
            point_size++;
        }
    }
}





void GlobalMap::lidarCloudCallback(const sensor_msgs::PointCloud2ConstPtr &point){
    static pcl::PointCloud<pcl::PointXYZ>::Ptr local_cloud_3d(new pcl::PointCloud<pcl::PointXYZ>);
    local_cloud_3d->clear();


    pcl::fromROSMsg(*point, *local_cloud_3d);

    /* 点云区域筛选 */
    pcl::PointXYZ pt;
    pcl::PointXYZ pt_3d;
    int point_size = 0;
    //获取局部点云
    for (int i = 0; i < (int)local_cloud_3d->points.size(); i++)
    {
        pt_3d = local_cloud_3d->points[i];
        if (pt_3d.z > m_2d_search_height_low && pt_3d.z < m_2d_search_height_high && (abs(pt_3d.x - odom_position(0)) < search_radius) && (abs(pt_3d.y - odom_position(1)) < search_radius))
        {
            pt.x = pt_3d.x;
            pt.y = pt_3d.y;
            pt.z = pt_3d.z;
            m_local_cloud->points.push_back(pt);
            point_size++;
        }
    }

    localPointCloudToObstacle(*m_local_cloud, true, odom_position);
    m_local_cloud->clear();
//    static std::queue<int> old_cloud_num;
//    static int record_times = 0;
//    old_cloud_num.push(point_size);
//    reverse(m_local_cloud->points.begin(), m_local_cloud->points.end());
//    if (++record_times > 4)
//    {
//        int abandon_num = old_cloud_num.front();
//        for (int i = 0; i < abandon_num; i++)
//        {
//            m_local_cloud->points.pop_back();
//        }
//        old_cloud_num.pop();
//    }
//    reverse(m_local_cloud->points.begin(), m_local_cloud->points.end());
}





/**
 * @brief	利用点云坐标为栅格地图设置障碍物
 *          （二维规划下只取指定高度范围的点云生成一层障碍物）
 * @param
 */
void GlobalMap::localPointCloudToObstacle(const pcl::PointCloud<pcl::PointXYZ> &cloud, const bool &swell_flag, const Eigen::Vector3d current_position, const int raycast_num)
{
    current_position_index = coord2gridIndex(current_position);  // 当前位置的栅格坐标

    pcl::PointXYZ pt;
    memset(l_data, 0, GLXY_SIZE * sizeof(uint8_t));  // TODO 这里直接reset，因此桥洞标志位需要不同的reset方式
    resetUsedGrids();
    //是否膨胀
    if (!swell_flag){

        for (int idx = 0; idx < (int)cloud.points.size(); idx++){
            pt = cloud.points[idx];
            localSetObs(pt.x, pt.y, pt.z);
        }
    }
    else{
        /* 膨胀处理 */
        for (int idx = 0; idx < (int)cloud.points.size(); idx++){
            pt = cloud.points[idx];
            int swell_num = (int)(m_robot_radius_dash / m_resolution);
            if (pt.x < gl_xl || pt.y < gl_yl || pt.z < gl_zl ||
                pt.x >= gl_xu || pt.y >= gl_yu || pt.z >= gl_zu)
                continue;
            int idx_x = static_cast<int>((pt.x - gl_xl) * m_inv_resolution);
            int idx_y = static_cast<int>((pt.y - gl_yl) * m_inv_resolution);
            int idx_z = static_cast<int>((0 - gl_zl) * m_inv_resolution);
            bool in_bridge_cave = false;
//            ROS_ERROR("ERROR2");
            //存在桥洞
            if(!GridNodeMap[idx_x][idx_y]->exist_second_height_||!GridNodeMap[idx_x][idx_y]->second_local_swell_){  // 非桥洞区域正常判断
                if (GridNodeMap[idx_x][idx_y]->height_ + m_height_threshold > pt.z || GridNodeMap[idx_x][idx_y]->height_ + 1.0 < pt.z)
                    continue;

            } else if(GridNodeMap[idx_x][idx_y]->second_local_swell_){  // 如果在桥洞里用一样的处理，只不过地图预设高度为second_height
                in_bridge_cave = true;
                if (GridNodeMap[idx_x][idx_y]->second_height_ + m_height_threshold > pt.z || GridNodeMap[idx_x][idx_y]->second_height_ + m_height_sencond_high_threshold < pt.z)
                    continue;
            }
            if(isOccupied(idx_x, idx_y, idx_z, false)){  // TODO 这里暂时处理为已经全局地图下被占用的情况下不再重新膨胀
                continue;
            }

            /// TODO 膨胀这里有点问题2024.6.18
            /// 通过高度地图阈值筛选之后膨胀设计全局地图的占用栅格数据，注意我们不需要设置局部地图高度
            for (int i = -swell_num; i <= swell_num; i++){
                for (int j = -swell_num; j <= swell_num; j++){
                    double temp_x = i * m_resolution + pt.x;
                    double temp_y = j * m_resolution + pt.y;
                    double temp_z = pt.z;

                    int idx = static_cast<int>((temp_x - gl_xl) * m_inv_resolution);
                    int idy = static_cast<int>((temp_y - gl_yl) * m_inv_resolution);
                    idx = std::max(idx, 0);
                    idy = std::max(idy, 0);
                    // 膨胀后的点要判断是不是在桥洞区域，以及如果是非桥洞点膨胀得到的点都不应该视为占用
                    if(GridNodeMap[idx_x][idx_y]->exist_second_height_ && !GridNodeMap[idx][idy]->exist_second_height_){
                        continue;
                    }

                    if(!GridNodeMap[idx_x][idx_y]->exist_second_height_ && GridNodeMap[idx][idy]->exist_second_height_){
                        continue;
                    }
                    localSetObs(temp_x, temp_y, temp_z);
                    /* 利用点云坐标为栅格地图设置障碍物 */
                }
            }
        }
    }
}


void GlobalMap::odometryCallback(const nav_msgs::OdometryConstPtr &state)
{
    geometry_msgs::Pose pose;
    pose = state->pose.pose;
    odom_position(0) = pose.position.x;
    odom_position(1) = pose.position.y;
    odom_position(2) = pose.position.z - 0.1;

    double x = pose.orientation.x;
    double y = pose.orientation.y;
    double z = pose.orientation.z;
    double w = pose.orientation.w;

    double siny_cosp = +2.0 * (w * z + x * y);
    double cosy_cosp = +1.0 - 2.0 * (y * y + z * z);

    odom_posture(2) = atan2f(siny_cosp, cosy_cosp);
    odom_posture(1) = asin(2 * (w * y - x * z));
    odom_posture(0) = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
}

void GlobalMap::gazeboPosCallback(const gazebo_msgs::ModelStatesConstPtr &state)
{
//    ROS_WARN("start gazebo position callback");
    int robot_namespace_id = 0;
    for(int i = 0;i < state->name.size();i++){
        if(state->name[i] == "mbot"){
            robot_namespace_id = i;
            break;
        }
    }
    odom_position(0) = state->pose[robot_namespace_id].position.x;
    odom_position(1) = state->pose[robot_namespace_id].position.y;
    odom_position(2) = state->pose[robot_namespace_id].position.z;

    double x = state->pose[robot_namespace_id].orientation.x;
    double y = state->pose[robot_namespace_id].orientation.y;
    double z = state->pose[robot_namespace_id].orientation.z;
    double w = state->pose[robot_namespace_id].orientation.w;
    double siny_cosp = +2.0 * (w * z + x * y);
    double cosy_cosp = +1.0 - 2.0 * (y * y + z * z);

    odom_posture(2) = atan2f(siny_cosp, cosy_cosp);
    odom_posture(1) = asin(2 * (w * y - x * z));
    odom_posture(0) = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
}

void GlobalMap::gazeboCloudCallback(const sensor_msgs::PointCloud2ConstPtr &point)
{
//    ROS_WARN("start gazebo pointcloud callback");
    static pcl::PointCloud<pcl::PointXYZ>::Ptr local_cloud_3d(new pcl::PointCloud<pcl::PointXYZ>);
    local_cloud_3d->clear();
    m_local_cloud->clear();

    pcl::fromROSMsg(*point, *local_cloud_3d);

    Eigen::Matrix3d R_x; // 计算旋转矩阵的X分量
    R_x << 1, 0, 0,
            0, cos(odom_posture[0]), -sin(odom_posture[0]),
            0, sin(odom_posture[0]), cos(odom_posture[0]);

    Eigen::Matrix3d R_y; // 计算旋转矩阵的Y分量
    R_y << cos(odom_posture[1]), 0, sin(odom_posture[1]),
            0, 1, 0,
            -sin(odom_posture[1]), 0, cos(odom_posture[1]);

    Eigen::Matrix3d R_z; // 计算旋转矩阵的Z分量
    R_z << cos(odom_posture[2]), -sin(odom_posture[2]), 0,
            sin(odom_posture[2]), cos(odom_posture[2]), 0,
            0, 0, 1;
    Eigen::Matrix3d R = R_z * R_y * R_x;

    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = R;
    transform.block<3, 1>(0, 3) = odom_position;

    // 定义输出点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 进行坐标系转换
    pcl::transformPointCloud(*local_cloud_3d, *output_cloud, transform);

    clock_t time_1 = clock();

    /* 点云区域筛选 */
    pcl::PointXYZ pt;
    pcl::PointXYZ pt_3d;
    int point_size = 0;

    for (int i = 0; i < (int) output_cloud->points.size(); i++)
    {
        pt_3d = output_cloud->points[i];
        /// 在范围内的点云
        if (pt_3d.z > m_2d_search_height_low && pt_3d.z < m_2d_search_height_high && (abs(pt_3d.x - odom_position(0)) < search_radius) && (abs(pt_3d.y - odom_position(1)) < search_radius))
        {
            pt.x = pt_3d.x;
            pt.y = pt_3d.y;
            pt.z = pt_3d.z + 0.2;
            m_local_cloud->points.push_back(pt);
            point_size++;
        }
    }

    clock_t time_2 = clock();
//    ROS_WARN("Time in local point  is %lf ms", (double)(time_2 - time_1) * 1000 / CLOCKS_PER_SEC);
//    ROS_WARN("output_cloud point size is  %d ", (int)output_cloud->points.size());
//    ROS_WARN("m_local_cloud point size is  %d ", (int)m_local_cloud->points.size());

    localPointCloudToObstacle(*m_local_cloud, true, odom_position);

//    static std::queue<int> old_cloud_num;
//    static int record_times = 0;
//    old_cloud_num.push(point_size);
//    reverse(m_local_cloud->points.begin(), m_local_cloud->points.end());
//    if (++record_times > 12)
//    {
//        int abandon_num = old_cloud_num.front();
//        for (int i = 0; i < abandon_num; i++)
//        {
//            m_local_cloud->points.pop_back();
//        }
//        old_cloud_num.pop();
//    }
//    reverse(m_local_cloud->points.begin(), m_local_cloud->points.end());

}
/**
 * @brief	设置局部障碍物（将障碍物信息保存到路径规划器里）
 *          膨胀处理（防撞设计）
 * @param   组成障碍物空间点坐标
*/
void GlobalMap::localSetObs(const double coord_x, const double coord_y, const double coord_z)
{   // 其实这里不用改的,桥洞占用和其他地方占用是不同的数据格式，设计reset时是不一样的
    if (coord_x < gl_xl || coord_y < gl_yl || coord_z < gl_zl ||
        coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu)
        return;
    // ROS_WARN("gl_zl %f", gl_zl);

    /* 用障碍物空间点坐标找到其所对应的栅格 */
    int idx_x = static_cast<int>((coord_x - gl_xl) * m_inv_resolution);
    int idx_y = static_cast<int>((coord_y - gl_yl) * m_inv_resolution);
    int idx_z = static_cast<int>((0 - gl_zl) * m_inv_resolution);

    if(GridNodeMap[idx_x][idx_y]->exist_second_height_){
        // 在桥洞位置，我们需要根据局部点云的高度和地图的高度差来判断是否桥洞被占用了还是高地被占用了
        if(coord_z > GridNodeMap[idx_x][idx_y]->second_height_ + m_height_threshold && coord_z < GridNodeMap[idx_x][idx_y]->second_height_ + m_height_sencond_high_threshold){  // 如果在桥洞里就设置障碍物
            GridNodeMap[idx_x][idx_y]->second_local_occupancy_ = true;  //桥下占用
        }else if(coord_z > GridNodeMap[idx_x][idx_y]->height_ + m_height_threshold && coord_z < GridNodeMap[idx_x][idx_y]->height_ + 0.3){  // 如果在桥上则在l_data中设置障碍物
            l_data[idx_x * GLY_SIZE + idx_y] = 1; // 桥上占用，在扫到桥洞前沿顶的时候膨胀可能会导致膨胀到外边，外边也是认为桥洞区域因此需要严格设置顶高的阈值(目前0.4)
//            std::cout<<"coord_z： "<<coord_z<<std::endl;
//            std::cout<<"GridNodeMap[idx_x][idx_y]->height: "<<GridNodeMap[idx_x][idx_y]->height<<std::endl;
        }
    }else{
        /* 将障碍物映射为容器里值为1的元素,根据这里的值做raycast process */
        l_data[idx_x * GLY_SIZE + idx_y] = 1;
    }
}


void GlobalMap::resetUsedGrids()
{
    for (int i = 0; i < GLX_SIZE; i++)
        for (int j = 0; j < GLY_SIZE; j++)
        {
            resetGrid(GridNodeMap[i][j]);
        }
}

inline void GlobalMap::resetGrid(GridNodePtr ptr)
{
    ptr->id_ = 0;
    ptr->cameFrom_ = NULL;
    ptr->gScore_ = inf;
    ptr->fScore_ = inf;
    ptr->second_local_occupancy_ = false;
}


Eigen::Vector3d GlobalMap::gridIndex2coord(const Eigen::Vector3i &index)
{
    Eigen::Vector3d pt;

    pt(0) = ((double)index(0) + 0.5) * m_resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * m_resolution + gl_yl;
    pt(2) = ((double)index(2) + 0.5) * m_resolution + gl_zl;

    return pt;
}

Eigen::Vector3i GlobalMap::coord2gridIndex(const Eigen::Vector3d &pt) {
    Eigen::Vector3i idx;
    idx << std::min(std::max(int((pt(0) - gl_xl) * m_inv_resolution), 0), GLX_SIZE - 1),
            std::min(std::max(int((pt(1) - gl_yl) * m_inv_resolution), 0), GLY_SIZE - 1),
            std::min(std::max(int((pt(2) - gl_zl) * m_inv_resolution), 0), GLZ_SIZE - 1);

    return idx;
}

void GlobalMap::coord2gridIndex(double &pt_x, double &pt_y, double &pt_z, int &x_idx, int &y_idx, int &z_idx)
{
    x_idx = std::min(std::max(int((pt_x - gl_xl) * m_inv_resolution), 0), GLX_SIZE - 1);
    y_idx = std::min(std::max(int((pt_y - gl_yl) * m_inv_resolution), 0), GLY_SIZE - 1);
    z_idx = std::min(std::max(int((pt_z - gl_zl) * m_inv_resolution), 0), GLZ_SIZE - 1);
}

