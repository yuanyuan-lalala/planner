#pragma once

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include"memory"
// #include "backward.hpp"

const double inf = 1e6; 
struct GridNode;
using GridNodePtr = std::shared_ptr<GridNode>;  

struct OccupancyNode;
using OccupancyNodePtr = std::shared_ptr<OccupancyNode>; 


struct GridNode
{
  enum class setType{
		UNEXPLORE,
		OPENSET,
    CLOSEDSET
	};

  setType m_set_type;
  int m_id;                   // 1--> open set, -1 --> closed set, 0 --> 还未被拓展到
  double m_height;
  Eigen::Vector3d m_coord; // 节点坐标（所处栅格的中心点坐标）
  Eigen::Vector3i m_dir;   // direction of expanding
  Eigen::Vector3i m_index; // 栅格的ID号

  double m_gScore, m_fScore; // 路径得分和启发得分
  
  bool exist_second_height_ = false; // 存在桥洞，高度及是否被占据
  double second_height_ = 0.08; //第二高度阈值 8cm
  bool second_local_occupancy_ = false;
  bool second_local_swell_ = false;
  double m_visibility = 0.0;

  GridNodePtr m_cameFrom; // 父节点
  std::multimap<double, GridNodePtr>::iterator nodeMapIt;

  GridNode(Eigen::Vector3i index, Eigen::Vector3d coord)
  {
    m_id = 0;
    m_height = -1.0;
    m_index = index;
    m_coord = coord;
    m_dir = Eigen::Vector3i::Zero();

    m_gScore = inf;
    m_fScore = inf;
    exist_second_height_ = false;
    m_cameFrom = nullptr;
  }

    GridNode(int x, int y, int z)
    {
        m_id = 1;
        m_height = -1.0;
        m_index(0) = x;
        m_index(1) = y;
        m_index(2) = z;
        m_dir = Eigen::Vector3i::Zero();

        m_gScore = inf;
        m_fScore = inf;
        m_cameFrom = nullptr;
        exist_second_height_ = false;
    }

    bool operator<(const GridNode & node) const
    {
      return (node.m_index(0) < m_index(0)) || ((node.m_index(0) == m_index(0)) &&(node.m_index(1) < m_index(1)));
    }

  GridNode(){};
  ~GridNode(){};
};

struct OccupancyNode
{
    Eigen::Vector3i m_index;
    bool m_unknown;
    bool m_occ;
    int m_count;
    int m_raycast_num;

    OccupancyNode(Eigen::Vector3i index, bool unknown)
    {
        m_index = index;
        m_unknown = unknown;
        m_occ = false;
        m_count = 0;
        m_raycast_num = 0;
    }
    OccupancyNode(){};
    ~OccupancyNode(){};
};
