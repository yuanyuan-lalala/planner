#include "global.h"

/** 定义SlaverSerial的静态成员 **/
std::unique_ptr<SlaverSerial> Global::slaver_serial;
std::vector<float> Global::slaver_serial_data;
SlaverSerial::DATA_TYPE Global::slaver_serial_data_type;

ros::Subscriber Global::speed_sub;
ros::Subscriber Global::perception_sub;
ros::Subscriber Global::xtl_flag_sub;
ros::Subscriber Global::attack_engineer_sub;
ros::Subscriber Global::global_path_sub;
ros::Subscriber Global::global_object_sub;

ros::Publisher Global::arrive_goal_pub;
ros::Publisher Global::user_interaction_arrival_goal_pub;
ros::Publisher Global::wheel_state_pub;
ros::Publisher Global::detect_enemy_pub; // 辅瞄数据
ros::Publisher Global::buff_status_pub;
ros::Publisher Global::damage_feedback_pub;
ros::Publisher Global::eco_status_pub;
ros::Publisher Global::game_status_pub;
ros::Publisher Global::referee_warn_pub;
ros::Publisher Global::robot_buff_pub;
ros::Publisher Global::robot_RFID_pub;
ros::Publisher Global::robot_HP_pub;
ros::Publisher Global::robot_status_pub;
ros::Publisher Global::shoot_feedback_pub;
ros::Publisher Global::user_interaction_pub;
ros::Publisher Global::UWB_location_pub;
ros::Publisher Global::robot_command_pub;
ros::Publisher Global::xtl_ready_flag_pub;
ros::Publisher Global::robot_pos_pub;

enum Global::ENEMY_COLOR Global::enemy_color;
uint8_t Global::enemy_id;
float Global::enemy_x;
float Global::enemy_y;
float Global::enemy_z;
float Global::enemy_value;
float Global::sentry_location_x;
float Global::sentry_location_y;
float Global::target_location_x;
float Global::target_location_y;

float Global::angle_target;
float Global::angle_current;
float Global::line_speed;

uint8_t Global::xtl_flag;
uint8_t Global::in_bridge = false;
std::vector<float> Global::global_objects;
std::vector<uint8_t> Global::global_id;

ros::Subscriber Global::sentry_location;

Referee Global::referee;

/**
 * 全局参数初始化
 */
void Global::paramInit(void) {
    /* 参数初始化 */
    enemy_color = Global::BLUE;
    slaver_serial_data_type = SlaverSerial::NONE;
}

void Global::rcvGlobalObjectsCallBack(const sentry_msgs::objects &msg){
    ROS_WARN("receive global cam data");
    global_objects.clear();
    global_id.clear();
    for(int i = 0; i < msg.n_target; i++) {
        global_id.push_back(msg.objects[i].id);
        global_objects.push_back(msg.objects[i].x);
        global_objects.push_back(msg.objects[i].y);
    }

    std::vector<float> float_vec;
    std::vector<unsigned char> char_vec;

    float_vec.push_back(Global::line_speed * cos(Global::angle_target)); // vx anglr vy
    float_vec.push_back(Global::angle_current);
    float_vec.push_back(Global::line_speed * sin(Global::angle_target));

    float_vec.push_back(Global::enemy_x);
    float_vec.push_back(Global::enemy_y);
    float_vec.push_back(Global::enemy_z);
    float_vec.push_back(Global::enemy_value);
    float_vec.push_back(Global::sentry_location_x);
    float_vec.push_back(Global::sentry_location_y);
    float_vec.push_back(Global::target_location_x);
    float_vec.push_back(Global::target_location_y);

    char_vec.push_back(Global::enemy_id);
    char_vec.push_back(Global::xtl_flag);
    char_vec.push_back(Global::in_bridge);

    int n_target = global_objects.size() / 2;  // 只发四个目标
    if(n_target >= 4){
        for(int i = 0; i < 4; i++){
            float_vec.push_back(Global::global_objects[2 * i + 0]);
            float_vec.push_back(Global::global_objects[2 * i + 1]);
            char_vec.push_back(Global::global_id[i]);
        }
    }else{
        for(int i = 0; i < n_target; i++){
            float_vec.push_back(Global::global_objects[2 * i + 0]);
            float_vec.push_back(Global::global_objects[2 * i + 1]);
            char_vec.push_back(Global::global_id[i]);
        }
        for(int i = 0; i < 4 - n_target; i++){
            float_vec.push_back(0.0);
            float_vec.push_back(0.0);
            char_vec.push_back(0);
        }
    }

    /* 发送期望速度到下位机 */
    Global::slaver_serial->send(float_vec, char_vec);
}

/**
 * @brief       目标速度回调函数
 * @param       speed 目标速度
 *              speed.line_speed 线速度
 *              speed.angle_speed 角速度
 * @attention   此函数中调用串口发送将目标速度发送给下位机
 */
void Global::rcvSpeedCallBack(const planner::slaver_speed & speed) {

    Global::angle_target = speed.angle_target;
    Global::angle_current = speed.angle_current;
    Global::line_speed = speed.line_speed;
    Global::xtl_flag = speed.xtl_flag;
    Global::in_bridge = speed.in_bridge;

    std::vector<float> float_vec;
    std::vector<unsigned char> char_vec;

    float_vec.push_back(Global::line_speed * cos(Global::angle_target)); // vx anglr vy
    float_vec.push_back(Global::angle_current);
    float_vec.push_back(Global::line_speed * sin(Global::angle_target));
//    float_vec.push_back(Global::angle_target); // vx anglr vy
//    float_vec.push_back(Global::angle_current);
//    float_vec.push_back(Global::line_speed);

    float_vec.push_back(Global::enemy_x);
    float_vec.push_back(Global::enemy_y);
    float_vec.push_back(Global::enemy_z);
    float_vec.push_back(Global::enemy_value);
    float_vec.push_back(Global::sentry_location_x);
    float_vec.push_back(Global::sentry_location_y);
    float_vec.push_back(Global::target_location_x);
    float_vec.push_back(Global::target_location_y);

    char_vec.push_back(Global::enemy_id);
    char_vec.push_back(Global::xtl_flag);
    char_vec.push_back(Global::in_bridge);

    int n_target = global_objects.size() / 2;  // 只发四个目标
    if(n_target >= 4){
        for(int i = 0; i < 4; i++){
            float_vec.push_back(Global::global_objects[2 * i + 0]);
            float_vec.push_back(Global::global_objects[2 * i + 1]);
            char_vec.push_back(Global::global_id[i]);
        }
    }else{
        for(int i = 0; i < n_target; i++){
            float_vec.push_back(Global::global_objects[2 * i + 0]);
            float_vec.push_back(Global::global_objects[2 * i + 1]);
            char_vec.push_back(Global::global_id[i]);
        }
        for(int i = 0; i < 4 - n_target; i++){
            float_vec.push_back(0.0);
            float_vec.push_back(0.0);
            char_vec.push_back(0);
        }
    }


    /* 发送期望速度到下位机 */
    Global::slaver_serial->send(float_vec, char_vec);
    static int times = 0;
    ROS_WARN("%d th sentry command (a_t, a_c, s_t) = (%.2%, %.2f, %.2f)\n", times++, float_vec[0], float_vec[1], float_vec[2]);
    std::cout<<"float_vec.size(): "<<float_vec.size()<<" char_vec.size(): "<<char_vec.size()<<std::endl;
}

void Global::rcvLocationCallBack(const nav_msgs::Odometry &state)
{
    Global::sentry_location_x = state.pose.pose.position.x;
    Global::sentry_location_y = state.pose.pose.position.y;
}


/**
 * @brief       敌方位置回调函数
 * @param       
 * @attention   
 */
void Global::rcvAttackCallBack(const sentry_msgs::object & enemy)
{
    Global::enemy_id = enemy.id;
    Global::enemy_x = enemy.x;
    Global::enemy_y = enemy.y;
    Global::enemy_z = enemy.z;
    Global::enemy_value = enemy.attack_value;

    std::vector<float> float_vec;
    std::vector<unsigned char> char_vec;

    float_vec.push_back(Global::line_speed * cos(Global::angle_target)); // vx anglr vy
    float_vec.push_back(Global::angle_current);
    float_vec.push_back(Global::line_speed * sin(Global::angle_target));
//    float_vec.push_back(Global::angle_target); // vx anglr vy
//    float_vec.push_back(Global::angle_current);
//    float_vec.push_back(Global::line_speed);
    float_vec.push_back(Global::enemy_x);
    float_vec.push_back(Global::enemy_y);
    float_vec.push_back(Global::enemy_z);
    float_vec.push_back(Global::enemy_value);
    float_vec.push_back(Global::sentry_location_x);
    float_vec.push_back(Global::sentry_location_y);
    float_vec.push_back(Global::target_location_x);
    float_vec.push_back(Global::target_location_y);

    char_vec.push_back(Global::enemy_id);
    char_vec.push_back(Global::xtl_flag);
    char_vec.push_back(Global::in_bridge);
//    ROS_WARN("start slaver");

    int n_target = global_objects.size() / 2;  // 只发四个目标
    if(n_target >= 4){
        for(int i = 0; i < 4; i++){
            float_vec.push_back(Global::global_objects[2 * i + 0]);
            float_vec.push_back(Global::global_objects[2 * i + 1]);
            char_vec.push_back(Global::global_id[i]);
        }
    }else{
        for(int i = 0; i < n_target; i++){
            float_vec.push_back(Global::global_objects[2 * i + 0]);
            float_vec.push_back(Global::global_objects[2 * i + 1]);
            char_vec.push_back(Global::global_id[i]);
        }
        for(int i = 0; i < 4 - n_target; i++){
            float_vec.push_back(0.0);
            float_vec.push_back(0.0);
            char_vec.push_back(0);
        }
    }

    /* 发送期望速度到下位机 */
    Global::slaver_serial->send(float_vec, char_vec);

    // ROS_WARN("%d th preception object (%.2f, %.2f, %.2f, %.2f, %.2f)\n", (float)Global::enemy_id,
    // Global::enemy_x, Global::enemy_y, Global::enemy_z, Global::enemy_value);
}

void Global::rcvTargetCallBack(const geometry_msgs::Point &path)
{
    if (path.x > 30 || path.y > 18 || path.x < 0 || path.y < 0)
        return;

    target_location_x = path.x;
    target_location_y = path.y;

    std::vector<float> float_vec;
    std::vector<unsigned char> char_vec;

    float_vec.push_back(Global::line_speed * cos(Global::angle_target)); // vx anglr vy
    float_vec.push_back(Global::angle_current);
    float_vec.push_back(Global::line_speed * sin(Global::angle_target));
    float_vec.push_back(Global::enemy_x);
    float_vec.push_back(Global::enemy_y);
    float_vec.push_back(Global::enemy_z);
    float_vec.push_back(Global::enemy_value);
    float_vec.push_back(Global::sentry_location_x);
    float_vec.push_back(Global::sentry_location_y);
    float_vec.push_back(Global::target_location_x);
    float_vec.push_back(Global::target_location_y);

    char_vec.push_back(Global::enemy_id);
    char_vec.push_back(Global::xtl_flag);
    char_vec.push_back(Global::in_bridge);

    int n_target = global_objects.size() / 2;  // 只发四个目标

    if(n_target >= 4){
        for(int i = 0; i < 4; i++){
            float_vec.push_back(Global::global_objects[2 * i + 0]);
            float_vec.push_back(Global::global_objects[2 * i + 1]);
            char_vec.push_back(Global::global_id[i]);
        }
    }else{
        for(int i = 0; i < n_target; i++){
            float_vec.push_back(Global::global_objects[2 * i + 0]);
            float_vec.push_back(Global::global_objects[2 * i + 1]);
            char_vec.push_back(Global::global_id[i]);
        }
        for(int i = 0; i < 4 - n_target; i++){
            float_vec.push_back(0.0);
            float_vec.push_back(0.0);
            char_vec.push_back(0);
        }
    }

    /* 发送期望速度到下位机 */
    Global::slaver_serial->send(float_vec, char_vec);

}

void Global::rcvXtlFlagCallBack(const std_msgs::Bool & msg)
{
    bool ready_xtl = msg.data;
    std_msgs::Bool ready_xtl_flag;
    ready_xtl_flag.data = ready_xtl;
    ros::Duration(0.5).sleep();
    Global::xtl_ready_flag_pub.publish(ready_xtl_flag);
}

