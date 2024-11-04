#pragma once
#include <memory>
#include <iostream>
#include <Eigen/Dense>
#include "optimizer/cubic_spline_optimizer.hpp"  
#include "optimizer/bspline_optimizer.hpp"  
#include "optimizer/optimizer_algorithm.hpp"

#include "vector"

class OptimizerManager {
public:
    // 定义优化算法的类型
    enum class AlgorithmType { CUBICSPLINE, BSPLINE };

    // 构造函数：根据指定的类型创建对应的优化器
    OptimizerManager(AlgorithmType type,std::shared_ptr<GlobalMap> global_map) {
        m_global_map = global_map;
        switch (type) {
            case AlgorithmType::CUBICSPLINE:
                optimizer = std::make_unique<CubicSplineOptimizer>(m_global_map);
                break;
            case AlgorithmType::BSPLINE:
                optimizer = std::make_unique<BsplineOptimizer>(m_global_map);
                break;
            default:
                std::cerr << "Unknown algorithm type." << std::endl;
                break;
        }
    }

    // 执行优化
    void executeOptimization() {
        if (optimizer) {
            optimizer->optimize();
           
        } else {
            std::cerr << "Optimizer not set." << std::endl;
        }
    }
   
    // 设置优化参数
    void setParameters(std::vector<Eigen::Vector3d> unoptimized_path,Eigen::Vector3d start_vel,double reference_speed) {
        optimizer->m_unoptimized_path = unoptimized_path;
        optimizer->m_start_vel = start_vel;
        optimizer->m_reference_speed = reference_speed; 
    }
    std::unique_ptr<OptimizerAlgorithm> optimizer;  // 优化器基类的指针，管理具体的优化器

private:
    
    Eigen::VectorXd parameters;            // 优化参数
    std::shared_ptr<GlobalMap> m_global_map;
    

    
    


};
