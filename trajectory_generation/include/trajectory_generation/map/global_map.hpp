#pragma once

#include "base_map.hpp"
class GlobalMap : public BaseMap{
    public:
    void initGridMap(ros::NodeHandle &nh, std::string occ_map_file, std::string bev_map_file, std::string distance_map_file,
                            double resolution, Eigen::Vector3d global_xyz_l, Eigen::Vector3d global_xyz_u,
                            int max_x_id, int max_y_id, int max_z_id, double robot_radius,
                            double _2d_search_height_low, double _2d_search_height_high, double search_radius);
    void lidarCloudCallback(const sensor_msgs::PointCloud2ConstPtr &point);
    void realsenseCloudCallback(const sensor_msgs::PointCloud2ConstPtr &point);
    void odometryCallback(const nav_msgs::OdometryConstPtr &state);
    
    void gazeboPosCallback(const gazebo_msgs::ModelStatesConstPtr &state);
    void gazeboCloudCallback(const sensor_msgs::PointCloud2ConstPtr &point);

    // void processSecondHeights();
    void bevMapToHeight(const cv::Mat bev_map, const cv::Mat occ_map1c);
    void occMaptoObstacle(const cv::Mat occ_map);
    void topoSampleMap(cv::Mat topo_map);
    
    void localPointCloudToObstacle(const pcl::PointCloud<pcl::PointXYZ> &cloud, const bool &swell_flag, const Eigen::Vector3d current_position, const int raycast_num);
    // void localSetObs(const double coord_x, const double coord_y, const double coord_z);

    
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_local_cloud;  // 局部点云地图
    std::vector<Eigen::Vector3d> second_heights_points;  // 处理桥洞高度
    std::vector<Eigen::Vector3d> topo_keypoint;
    std::vector<Eigen::Vector3d> topo_sample_map;

    
    float wObstacle = 1e5;  //不飞坡的话大一点保证安全

    
    ros::NodeHandle m_nh;
    Eigen::Vector3i current_position_index;

};