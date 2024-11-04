#pragma once
#include "optimizer/optimizer_algorithm.hpp"
#include "root_solver/cubic_spline.hpp"
#include "root_solver/lbfgs.hpp"

class CubicSplineOptimizer : public OptimizerAlgorithm{
    public:
    explicit CubicSplineOptimizer(std::shared_ptr<GlobalMap> global_map);
    void init(std::vector<Eigen::Vector3d>& unoptimized_path, Eigen::Vector3d start_vel, double desire_speed);
    void optimize() override;
    void optimizePath();
    void pathResample();
    std::vector<Eigen::Vector2d> getPath();
    void pathSample(std::vector<Eigen::Vector3d>& global_path, Eigen::Vector3d start_vel);
    void getGuidePath(Eigen::Vector2d start_vel, double radius);
    static double costFunction(void *ptr, const Eigen::VectorXd &x, Eigen::VectorXd &g);
    Eigen::Vector2d obstacleTerm(int idx, Eigen::Vector2d xcur, double &nearest_cost);
    
    
    
    double desire_velocity;
    Eigen::Vector2d headP;
    Eigen::Vector2d tailP;
    int pieceN; //路径数量
    Eigen::Matrix2Xd pathInPs;

    CubicSpline cubSpline;
    lbfgs::lbfgs_parameter lbfgs_params;
    
    

};