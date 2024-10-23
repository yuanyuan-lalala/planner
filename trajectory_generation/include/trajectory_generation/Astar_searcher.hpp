#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include"sensor_msgs/PointCloud2.h"
#include"global_map.hpp"
#include"pcl_conversions/pcl_conversions.h"
class AstarPathFinder
{

    public:
	

    void visLocalGridMap(const pcl::PointCloud <pcl::PointXYZ> &cloud, const bool swell_flag);
    void visGridMap();
    
    Eigen::Vector3d coordRounding(const Eigen::Vector3d &coord);
    Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d &pt);
    
    bool isLocalOccupied(const Eigen::Vector3i &index) const;
    bool isLocalOccupied(const int &idx_x, const int &idx_y, const int &idx_z) const;
    bool isOccupied(const Eigen::Vector3i &index) const;
    bool isOccupied(const int &idx_x, const int &idx_y, const int &idx_z) const;

    bool checkPathCollision(std::vector <Eigen::Vector3d> optimized_path, Eigen::Vector3d &collision_pos);

    Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i &index);

    std::vector<Eigen::Vector3d> smoothPath(std::vector<Eigen::Vector3d> src_path, bool continuous );
    std::vector<Eigen::Vector3d> smoothTopoPath(std::vector<Eigen::Vector3d> topo_path);
    
    Eigen::Vector3d getMidPoint(std::vector<Eigen::Vector3d> midpath);
    bool lineVisib(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, Eigen::Vector3d& colli_pt, double thresh);
    bool getNearPoint(Eigen::Vector3d headPos, Eigen::Vector3d tailPos, Eigen::Vector3d &best_point);
    bool findNeighPoint(Eigen::Vector3i path_point, Eigen::Vector3i &output_point, int check_num);


    bool isFree(const Eigen::Vector3i &index) const;
    bool isFree(const int &idx_x, const int &idx_y, const int &idx_z) const;
    
    int GLX_SIZE, GLY_SIZE, GLZ_SIZE; // 地图尺寸
	int GLXYZ_SIZE, GLYZ_SIZE;

	double m_resolution, m_inv_resolution; // 栅格长度以及1m内的栅格数目
	double gl_xl, gl_yl, gl_zl;			   // 地图的左下角坐标
	double gl_xu, gl_yu, gl_zu;			   // 地图的右上角坐标

	double m_2d_search_height_low;	// 2维规划障碍物搜索高度最低点
	double m_2d_search_height_high; // 2维规划障碍物搜索高度最高点
	double m_robot_radius;			// 机器人的碰撞检测直径

    

    std::shared_ptr<GlobalMap> global_map;
    ros::Publisher grid_map_vis_pub, local_grid_map_vis_pub;

};
