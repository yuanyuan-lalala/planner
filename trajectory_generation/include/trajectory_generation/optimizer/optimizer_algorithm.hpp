#pragma once
#include"Eigen/Core"
#include"vector"
#include "memory"
#include "map/global_map.hpp"

class OptimizerAlgorithm{
    public:
    OptimizerAlgorithm(std::shared_ptr<GlobalMap> global_map);
    virtual ~OptimizerAlgorithm() = default; 
    virtual void optimize() = 0; 
    std::vector<Eigen::Vector3d> m_unoptimized_path;
    std::vector<Eigen::Vector2d> m_sampled_path;  // 采样得到的初始轨迹
    std::vector<Eigen::Vector2d> m_final_path;

    std::vector<double> m_trapezoidal_time;
    std::shared_ptr<GlobalMap> m_global_map;

    Eigen::Vector3d m_start_vel;
    double m_reference_speed;

    std::vector<double> mid_distance;
    std::vector<double> weights;

    bool init_obs = false;
    bool init_vel = false;
    bool using_curv = false;
    
};