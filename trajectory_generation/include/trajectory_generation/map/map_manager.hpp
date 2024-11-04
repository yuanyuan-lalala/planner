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
#include "global_map.hpp"
#include "local_map.hpp"
#include "occupancy_map.hpp"
#include "topo_map.hpp"


class MapManager{

public:

    virtual ~MapManager(){};

    public:
    void initMaps(ros::NodeHandle &nh) {
        
        m_global_map = std::make_shared<GlobalMap>();
        m_local_map = std::make_shared<LocalMap>();
        m_topo_map = std::make_shared<TopoMap>();
        m_occupancy_map = std::make_shared<OccupancyMap>();
        
    }



    private:
    std::shared_ptr<GlobalMap> m_global_map;
    std::shared_ptr<LocalMap> m_local_map;
    std::shared_ptr<OccupancyMap> m_occupancy_map;
    std::shared_ptr<TopoMap> m_topo_map;


   
};
