#ifndef PLANNER_MANAGER_HPP
#define PLANNER_MANAGER_HPP
#include"ros/ros.h"
#include"Eigen/Core"
#include"Eigen/Dense"
#include"random"
#include "memory"
#include"global_map.hpp"
#include "path_smooth.hpp"
#include "reference_path.hpp"
#include"topo_searcher.hpp"
#include"Astar_searcher.hpp"


class PlannerManager{

public:
    void init(ros::NodeHandle& nh);
    bool pathFinding(const Eigen::Vector3d start_pt, const Eigen::Vector3d target_pt,
                     const Eigen::Vector3d start_vel);
    bool replanFinding(const Eigen::Vector3d start_pt, const Eigen::Vector3d target_pt,
                       const Eigen::Vector3d start_vel);
    void pubGlobalPlanningResult(std::vector<Eigen::Vector3d> nodes);
    bool AstarGlobalResult(const Eigen::Vector3d start_pt, const Eigen::Vector3d target_pt);

    // enum class teamColor{
    //     RED ,
    //     BLUE ,
    // };
    // teamColor sentryColor;


    typedef enum {
        RED = 0,
        BLUE,
    } teamColor;
    teamColor sentryColor;

    std::unique_ptr<PlannerManager> m_plannerManager;
    std::shared_ptr<GlobalMap> m_globalMap;
    std::unique_ptr<Smoother> m_smoother;
    std::unique_ptr<ReferenceSmooth> m_referenceSmooth;
    std::unique_ptr<TopoSearcher> m_topoPRM;
    std::unique_ptr<AstarPathFinder> m_astar_path_finder;
   
    // 参考路径的最大期望avw
    double reference_v_max;
    double reference_a_max;
    double reference_w_max;
    double reference_desire_speed;
    double reference_desire_speed_spinning;

    std::vector<std::vector<Eigen::Vector3d>> sample_path_set;
    std::random_device m_rd;
    std::default_random_engine m_eng;
    std::uniform_real_distribution<double> m_rand_pos;
    
    /// 小陀螺或者小虎步时的avw
    double reference_v_spinning_max;
    double reference_a_spinning_max;
    double reference_w_spinning_max;

    /* 相关标志位 */
    bool is_spinning;    // 电控陀螺标志位
    bool spinning_flag; // 规划陀螺标志位

    /** 地图相关参数：四个初始地图及地图参数 **/
    std::string map_file_path;
    std::string occ_file_path;
    std::string bev_file_path;
    std::string distance_map_file_path;

    double map_resolution;
    double map_inv_resolution;
    double map_x_size;
    double map_y_size;
    double map_z_size;
    Eigen::Vector3d map_lower_point;  // 地图的左上角和右下角
    Eigen::Vector3d map_upper_point;
    double search_height_min;         // 局部点云可视范围
    double search_height_max;
    double search_radius;

    /* 路径查找相关起点终点和仿真特有的地图偏执*/
    Eigen::Vector3d start_point;
    Eigen::Vector3d target_point;
    Eigen::Vector3d map_offset;

    int grid_max_id_x;
    int grid_max_id_y;
    int grid_max_id_z;
    int raycast_num;
    int global_planning_times;
    bool current_pos_init_flag;
    bool visualization_flag;

    /* 放在replan中机器人相关位置，速度，yaw角速度，姿态状态 */
    Eigen::Vector3d robot_cur_position;
    Eigen::Quaterniond robot_cur_orientation;
    Eigen::Vector2d robot_cur_speed;
    double robot_line_speed = 0.0;
    double robot_angular = 0.0;
    double robot_cur_yaw = 0.0;

    /// 机器人点云膨胀半径，太大了不好哦
    double robot_radius;
    double robot_radius_dash;

    /* 相关标志位 */
    bool obstacle_swell_flag;
    bool obstacle_swell_vis_flag;

    /*全局规划结果与局部规划结果*/
    std::vector<Eigen::Vector3d> optimized_path;
    std::vector<Eigen::Vector3d> local_optimize_path;
    std::vector<Eigen::Vector3d> ref_trajectory;  // 可视化和重规划判断
    std::vector<Eigen::Vector3d> astar_path;
    std::vector<Eigen::Vector2d> final_path;
    std::vector<Eigen::Vector2d> final_path_temp;  // 临时可视化变量
    // std::vector<GraphNode::Ptr> global_graph;




private:








};



#endif