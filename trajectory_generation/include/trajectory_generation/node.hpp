#pragma once

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
// #include "backward.hpp"

const double inf = 1e6; 

struct GridNode;
using GridNodePtr = GridNode *;

struct OccupancyNode;
using OccupancyNodePtr = OccupancyNode*;

struct GridNode
{
  int id_;                   // 1--> open set, -1 --> closed set, 0 --> 还未被拓展到
  double height_;
  Eigen::Vector3d coord_; // 节点坐标（所处栅格的中心点坐标）
  Eigen::Vector3i dir_;   // direction of expanding
  Eigen::Vector3i index_; // 栅格的ID号

  double gScore_, fScore_; // 路径得分和启发得分
  
  bool exist_second_height_ = false; // 存在桥洞，高度及是否被占据
  double second_height_ = 0.08; //第二高度阈值 8cm
  bool second_local_occupancy_ = false;
  bool second_local_swell_ = false;
  double visibility_ = 0.0;

  GridNodePtr cameFrom_;  // 父节点
  std::multimap<double, GridNodePtr>::iterator nodeMapIt;

  GridNode(Eigen::Vector3i index, Eigen::Vector3d coord)
  {
    id_ = 0;
    height_ = -1.0;
    index_ = index;
    coord_ = coord;
    dir_ = Eigen::Vector3i::Zero();

    gScore_ = inf;
    fScore_ = inf;
    exist_second_height_ = false;
    cameFrom_ = NULL;
  }

    GridNode(int x, int y, int z)
    {
        id_ = 1;
        height_ = -1.0;
        index_(0) = x;
        index_(1) = y;
        index_(2) = z;
        dir_ = Eigen::Vector3i::Zero();

        gScore_ = inf;
        fScore_ = inf;
        cameFrom_ = NULL;
        exist_second_height_ = false;
    }

    bool operator<(const GridNode & node) const
    {
      return (node.index_(0) < index_(0)) || ((node.index_(0) == index_(0)) &&(node.index_(1) < index_(1)));
    }

  GridNode(){};
  ~GridNode(){};
};

struct OccupancyNode
{
    Eigen::Vector3i index_;
    bool unknown_;
    bool occ_;
    int count_;
    int m_raycast_num_;

    OccupancyNode(Eigen::Vector3i index, bool unknown)
    {
        index_ = index;
        unknown_ = unknown;
        occ_ = false;
        count_ = 0;
        m_raycast_num_ = 0;
    }
    OccupancyNode(){};
    ~OccupancyNode(){};
};
