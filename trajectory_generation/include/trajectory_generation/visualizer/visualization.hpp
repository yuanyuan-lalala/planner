#pragma once
#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include"Eigen/Core"
#include "Eigen/Dense"
#include "searcher/topo_searcher.hpp"
#include "map/global_map.hpp"

class Visualization
{
    public:
    void init(ros::NodeHandle& nh,std::shared_ptr<GlobalMap> global_map);
    void visAstarPath(std::vector<Eigen::Vector3d> nodes);
    void visObs(std::vector<std::vector<Eigen::Vector3d>> nodes);
    void visFinalPath(std::vector<Eigen::Vector3d> nodes);
    void visOptimizedPath(std::vector<Eigen::Vector2d> nodes);
    void visOptGlobalPath(const std::vector<Eigen::Vector3d>& nodes);
    void visCurPosition(const Eigen::Vector3d cur_pt);
    void visTargetPosition(const Eigen::Vector3d target_pt);
    void visAttackPoint(Eigen::Vector3d target_pos, Eigen::Vector3d target_vel);
    void visTopoPath(std::vector<std::vector<Eigen::Vector3d>> path);
    void visTopoPointGuard(std::vector<GraphNode::Ptr> global_graph);
    void visTopoPointConnection(std::vector<GraphNode::Ptr> global_graph);

    void visLocalGridMap(const pcl::PointCloud <pcl::PointXYZ> &cloud, const bool swell_flag);
    void visGridMap();
  

    ros::Publisher astar_path_vis_pub, optimized_path_vis_pub, cur_position_vis_pub, obs_vis_pub;
    ros::Publisher reference_path_vis_pub, final_path_vis_pub, target_position_vis_pub, final_line_strip_pub;
    ros::Publisher topo_position_guard_vis_pub, topo_position_connection_vis_pub, topo_line_vis_pub;
    ros::Publisher topo_path_point_vis_pub, topo_path_vis_pub, attack_target_vis_pub;

    ros::Publisher grid_map_vis_pub, local_grid_map_vis_pub;
    
    std::shared_ptr<GlobalMap> m_global_map;


};
