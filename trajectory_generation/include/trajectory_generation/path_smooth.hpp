#ifndef PATH_SMOOTH_HPP
#define PATH_SMOOTH_HPP
#include "memory"
#include"global_map.hpp"
#include "root_solver/cubic_spline.hpp"
#include"root_solver/lbfgs.hpp"

class Smoother{

public:
    void init(std::vector<Eigen::Vector3d>& global_path, Eigen::Vector3d start_vel, double desire_speed);
    void setGlobalMap(std::shared_ptr<GlobalMap> &global_map);
    void pathSample(std::vector<Eigen::Vector3d>& global_path, Eigen::Vector3d start_vel);
    void getGuidePath(Eigen::Vector2d start_vel, double radius = 0.5);
    void smoothPath();
    static double costFunction(void *ptr, const Eigen::VectorXd &x, Eigen::VectorXd &g);
    Eigen::Vector2d obstacleTerm(int idx, Eigen::Vector2d xcur, double &nearest_cost);
    void getObsEdge(Eigen::Vector2d xcur);
    bool getObsPosition(double start_x, double start_y, double start_z,
                    double edge_x, double edge_y, double edge_z, Eigen::Vector3d &obs_pt);
    bool isFree(const Eigen::Vector3i &index) const;
    bool isFree(const int &idx_x, const int &idx_y, const int &idx_z) const;
    void pathResample();

    std::vector<Eigen::Vector2d> getPath();
    std::vector<Eigen::Vector2d> getSamplePath();
    


    std::shared_ptr<GlobalMap> global_map_;

    std::vector<Eigen::Vector2d> path;  // 采样得到的初始轨迹
    std::vector<Eigen::Vector2d> finalpath;
    std::vector<std::vector<Eigen::Vector3d>> allobs;
    std::vector<Eigen::Vector3d> obs_coord;
    std::vector<double> mid_distance;
    std::vector<double> m_trapezoidal_time;

    lbfgs::lbfgs_parameter lbfgs_params;
    Eigen::Matrix2Xd pathInPs;


    double desire_veloity;
    Eigen::Vector2d headP;
    Eigen::Vector2d tailP;
    
    bool init_obs = false;
    bool init_vel = false;
    bool using_curv = false;

    int pieceN; //路径数量
    float wObstacle = 1e5;  //不飞坡的话大一点保证安全
    

    CubicSpline cubSpline;
    

private:




};
#endif