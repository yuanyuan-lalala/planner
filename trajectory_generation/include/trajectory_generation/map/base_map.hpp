#pragma once
#include"Eigen/Core"
#include "opencv2/opencv.hpp"
#include "ros/ros.h"
#include "node_utils.hpp"
#include"sensor_msgs/PointCloud2.h"
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include "pcl/point_cloud.h"
#include"pcl/point_types.h"
#include"pcl_conversions/pcl_conversions.h"
#include <pcl/common/transforms.h>


class BaseMap{

public:

    virtual ~BaseMap(){};

    Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i &index);
    Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d &pt);
    void coord2gridIndex(double &pt_x, double &pt_y, double &pt_z, int &x_idx, int &y_idx, int &z_idx);
    
    void setRadiusDash(const double dash_radius);
    cv::Mat swellOccMap(cv::Mat occ_map);

    virtual void setObs(const double coord_x, const double coord_y);
    virtual void localSetObs(const double coord_x, const double coord_y, const double coord_z);
    void getObsEdge(Eigen::Vector2d xcur);
    bool getObsPosition(double start_x, double start_y, double start_z,
                    double edge_x, double edge_y, double edge_z, Eigen::Vector3d &obs_pt);
    
    void setHeight(const double coord_x, const double coord_y, const double coord_z, const bool exist_heights);
    
    
    virtual void resetUsedGrids();
    void resetGrid(GridNodePtr ptr);


    virtual bool isOccupied(const int &idx_x, const int &idx_y, const int &idx_z, bool second_height) const;
    virtual bool isOccupied(const Eigen::Vector3i &index, bool second_height) const;
    virtual bool isLocalOccupied(const int &idx_x, const int &idx_y, const int &idx_z) const;
    virtual bool isLocalOccupied(const Eigen::Vector3i &index) const;
    virtual bool isFree(const int &idx_x, const int &idx_y, const int &idx_z) const;
    virtual bool isFree(const Eigen::Vector3i &index) const;

    double getHeight(const int idx_x, const int idx_y);


    Eigen::Vector3d coordRounding(const Eigen::Vector3d &coord);
   
    std::vector<uint8_t> m_global_obstacle_map;   // 全局地图的障碍物信息
    std::vector<uint8_t> m_local_obstacle_map; // 局部地图的障碍物信息

    std::vector<Eigen::Vector3d> obs_coord;
    std::vector<std::vector<Eigen::Vector3d>> allobs;

   
    std::shared_ptr<std::vector<std::vector<GridNodePtr>>> GridNodeMap;       // 全局地图
    std::shared_ptr<std::vector<std::vector<GridNodePtr>>> GridNodeLocalMap;  // 局部地图
    std::shared_ptr<std::vector<std::vector<OccupancyNodePtr>>> OccupancyMap;
    
    ros::NodeHandle m_node;
    ros::Subscriber m_local_point_sub, m_odometry_sub, gazebo_point_sub, gazebo_odometry_sub, m_realsense_sub;
    ros::Publisher global_map_pub;

    double m_height_bias;  // (height = m_height_interval / 255.0 + m_height_bias)
    double m_height_interval;
    double m_height_threshold;
    double m_height_sencond_high_threshold;

    double m_2d_search_height_low;	// 2维规划障碍物搜索高度最低点
    double m_2d_search_height_high; // 2维规划障碍物搜索高度最高点
    
    double m_robot_radius;			// 机器人的碰撞检测直径 建议<0.38
    double m_robot_radius_dash;		// 机器人的动态障碍物冲卡检测直径 建议<0.25
    

    double gl_xl, gl_yl, gl_zl;			   // 地图的左下角坐标
    double gl_xu, gl_yu, gl_zu;			   // 地图的右上角坐标
    double m_search_radius; 

    int GLX_SIZE, GLY_SIZE, GLZ_SIZE; // 地图尺寸
    int GLXYZ_SIZE, GLYZ_SIZE, GLXY_SIZE;
    double m_resolution, m_inv_resolution; // 栅格长度以及1m内的栅格数目

    Eigen::Vector3d odom_position;
    Eigen::Vector3d odom_posture;
    double search_radius;
};
