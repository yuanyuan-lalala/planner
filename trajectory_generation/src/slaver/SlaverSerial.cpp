/********************************************************

串口协议为 55 00 ID NUM NUM x float 00 AA

比如 功能ID为 03
发送数据为1个float 则协议为
===========================================================
55 00 | 03 | 01 | xx xx xx xx | xx xx xx xx | ..... | 00 AA
起始位  功能  计数  数<------------>据<-------------->位 结束位
===========================================================
该程序应用了boost asio 库
其中功能位接收数据时用于表示红蓝方，发送时用于表示是否找到目标

0000 0000 x x

*********************************************************/

#include "SlaverSerial.h"
#include "global.h"
#include "uart_protocol.h"

/**
 * 使用文件操作寻找 COM 口
 * @param
 * @return Device string
 */

std::string findDevice() {
    DIR *pDir;  ///文件浏览器，寻找 Linux 下以文件形式存在的设备
    struct dirent *ptr;  ///包含当前文件信息的结构体
    std::vector<std::string> files;
    if (!(pDir = opendir("/dev/"))) {  ///定位到 /dev/ 文件下
        return "faild";
    }
    const std::string path0 = "/dev/";
    std::string subFile;
    while ((ptr = readdir(pDir)) != 0) {  ///读取当前位置下所有文件的信息
        subFile = ptr->d_name;            ///文件名
        if (subFile.substr(0, subFile.size() - 1) ==
            "MYUSB") {  ///找到文件名中从头开始数大小位后边的字符串是否是
            /// ttyUSB
            files.emplace_back(
                    path0 + subFile);  ///使用 emplace_back （类似于 push_back）
        }
    }
    if (!files.empty()) {
        std::cout << "find serial:" << std::endl;
    } else {
        std::cerr << "Error:no serial find!" << std::endl;
        return "faild";
    }
    for (auto device : files) {
        std::cout << device << std::endl;
    }
    return files[0];
}
/**
 * 构造函数
 * @param str COM 寻找方式
 * @param baud_rate 波特率
 * @return
 */
SlaverSerial::SlaverSerial(const std::string str, int baud_rate) {
    if (str != "auto") {
        m_serialBase = new SerialBase(str.c_str(),
                                      baud_rate);  ///串口基层，负责收发具体数据
    } else {
        std::string device = findDevice();
        if (device == "faild") {
            exit(1);
        }
        m_serialBase = new SerialBase(device.c_str(),
                                      baud_rate);  ///串口基层，负责收发具体数据
    }
    MAX_RECEIVE_FLOAT_LENGTH = 20;
    FIRST_ONE = 0x55;  ///起始位置 1
    FIRST_TWO = 0x00;  ///起始位置 2

    LAST_ONE = 0x00;  ///倒数第二
    LAST_TWO = 0xAA;  ///倒数第一
}

SlaverSerial::~SlaverSerial() { delete m_serialBase; }
/**
 * 通过有限状态自动机来解码
 * @param buff 读入的字节流
 * @param received_length 所能够接受的字节流最长长度
 * @param num 浮点数据容器
 * @param receive_flag 判断的敌方的颜色
 * @return
 */
unsigned char SlaverSerial::decode(unsigned char *buff, size_t &received_length,
                          std::vector<float> &num) {
    static int flag = 1;                        ///接收状态机
    static std::vector<float> vec_num;      ///接收数据
    static int temp_num_length;
    unsigned char return_flag = 0;
    SlaverSerial::DATA_TYPE data_type = SlaverSerial::NONE;;// 接收数据的类型
    // for (size_t i = 0; i < received_length; i++) {
    //     ROS_WARN("%x\n", buff[i]);
    // }
//     ROS_WARN("Here 1\n");

    for (size_t i = 0; i < received_length; i++) {
        switch (flag) {
            case 1:  ///报头1 读取第一位起始位
                vec_num.clear();
                if (FIRST_ONE == buff[i]) {
                    flag = 2;  ///通过
                } else {
                    flag = 1;  ///丢弃
                    return_flag = 9;
                }
                break;
            case 2:  ///报头2 读取第二位起始位
                if (FIRST_TWO == buff[i]) {
                    flag = 3;  ///通过
                } else {
                    flag = 1;  ///丢弃
                    return_flag = 2;
                }
                break;
            case 3:  ///控制类型位
                if (0x09 == buff[i]) {
                    data_type = SlaverSerial::ENEMY_COLOR;
                    Global::enemy_color = Global::BLUE;
                    flag = 4;
                } else if (0x0A == buff[i]) {
                    data_type = SlaverSerial::ENEMY_COLOR;
                    Global::enemy_color = Global::RED;
                    flag = 4;
                } else if (0x01 == buff[i]) {
                    data_type = SlaverSerial::ARRIVE_GOAL;
                    flag = 4;
                } else if (0x00 == buff[i]) {
                    data_type = SlaverSerial::NONE;
                    flag = 4;
                } else {
                    flag = 1;  ///丢弃
                    return_flag = 3;
                }
                break;
            case 4:  ///浮点数长度
                temp_num_length = static_cast<int>(buff[i]);  ///转换
                if (temp_num_length < 0 && temp_num_length > MAX_RECEIVE_FLOAT_LENGTH)
                {
                    flag = 1;
                } else if (temp_num_length == 0)  ///有效但是数据为空，直接进入结束判断
                {
                    flag = 6;
                }
                else {
                    flag = 5;
                }
                
                break;
            case 5:
                float temp;
                if (decode_num(buff[i], temp)) { // 如果返回 true 表示数据有效
                    vec_num.push_back(temp);
                }  
                if (vec_num.size() == temp_num_length)  // 校验数据个数
                    flag = 6;
                break;
            case 6:
                if (LAST_ONE == buff[i])  // 第一结束位
                    flag = 7;             // 通过
                else
                {
                    flag = 1;  ///丢弃
                    return_flag = 6;
                }
                break;
            case 7:
                if (LAST_TWO == buff[i]) {  ///第二结束位，通过
                    num = vec_num;                     ///重载=的复制
                    return_flag = 1;                ///数据合法
                }
                flag = 1;  ///丢弃和重置
                break;
        }
    }

    if (return_flag != 1) {
        Global::slaver_serial_data_type = SlaverSerial::NONE;
    } else {
        Global::slaver_serial_data_type = data_type;
        sentry_msgs::DetectObject detect_object;
        geometry_msgs::Vector3 wheel_state;

        wheel_state.x = Global::slaver_serial_data[2];
        wheel_state.y = Global::slaver_serial_data[3];
        wheel_state.z = Global::slaver_serial_data[3];

        detect_object.id = Global::slaver_serial_data[4];
        detect_object.x = Global::slaver_serial_data[5];
        detect_object.y = Global::slaver_serial_data[6];
        detect_object.z = Global::slaver_serial_data[7];


        Global::wheel_state_pub.publish(wheel_state);
        Global::detect_enemy_pub.publish(detect_object);
    }

    if (Global::slaver_serial_data_type == SlaverSerial::ARRIVE_GOAL) {
        sentry_msgs::UserInteraction_arrival_goal userInteraction_arrival_goal;

        // if (Global::slaver_serial_data_type == SlaverSerial::ENEMY_COLOR) {
        nav_msgs::Path arrive_goal;
        arrive_goal.header.frame_id = std::string("world");
        arrive_goal.header.stamp = ros::Time::now();
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = Global::slaver_serial_data[0];
        pose.pose.position.y = Global::slaver_serial_data[1];
        pose.pose.position.z = 0;
        arrive_goal.poses.push_back(pose);
        userInteraction_arrival_goal.path_buff = arrive_goal;
        userInteraction_arrival_goal.target_id = 3;
        Global::arrive_goal_pub.publish(arrive_goal);
        Global::user_interaction_arrival_goal_pub.publish(userInteraction_arrival_goal);
        Global::slaver_serial_data_type = SlaverSerial::NONE;
    }
    
    return return_flag;
}

/**
 * 利用union的特性，写入usigned char的过程即是在写入float
 * @param buff 读入的字节
 * @param num 解码出的数字所存储的指针
 * @return 是否解析完毕
 */
bool SlaverSerial::decode_num(unsigned char buff, float &num) {
    static float2uchar temp_num;  ///转换结构
    static int pos = 0;           ///读取计数，每到 4 个为一组数据
    temp_num.ch[pos] = buff;
    pos++;
    if (pos == 4) {
        num = temp_num.f;
        pos = 0;
        return true;
    }
    return false;
}
/**
 * 发送接口
 * @param send_flag 功能控制位
 * @param num 数据容器
 * @return
 */
void SlaverSerial::send(std::vector<float> num_f, std::vector<unsigned char> num_c) {
    m_send_mutex.lock();  ///线程锁，保证多个线程发送只能单独使能一个

    int send_length = num_c.size() + num_f.size() * 4 + 6;  ///由于 float 一组是四个字节
    assert(send_length < 100);  ///限制单次发送大小为 100 字节数据
    unsigned char buff[100] = {};  ///发送缓冲区

    buff[0] = FIRST_ONE;  ///开头复制为两个字节起始位之一 000
    buff[1] = FIRST_TWO;  ///开头复制为两个字节起始位之一 111

    buff[2] = 0x01;         // 指令ID
    buff[3] = static_cast<unsigned char>(
            num_f.size());  // 需要发送的 float 组大小 size_t 转换为 uchar 类型 333

    for (int i = 0; i < num_f.size(); i++) {
        float2uchar temp;
        temp.f = num_f[i];
        // ROS_WARN("Here::: %f", temp.f);
        for (int j = 0; j < 4; j++)
            buff[i * 4 + 4 + j] = temp.ch[j];  /// 444 555 ... 通过共用体转换
    }
    for (int i = 0; i < num_c.size(); i++) {
        buff[num_f.size() * 4 + 4 + i] = num_c[i];
        // ROS_WARN("Here::: %d", num_c[i]);
    }
//    ROS_WARN("Byte length::: %d", send_length);

    buff[num_f.size() * 4 + num_c.size() + 4] = LAST_ONE;      ///结束位
    buff[num_f.size() * 4 + num_c.size() + 4 + 1] = LAST_TWO;  /// 结束位

    m_serialBase->send(buff, send_length);  ///发送

    m_send_mutex.unlock();  ///释放线程
}
/**
 * 接收接口
 * @param receive_flag 接收颜色位
 * @param num 数据容器
 * @return
 */
void SlaverSerial::receive(std::vector<float>& f_data) {
    while (1) {
        /* 接收次数 */
        static int rec_times = 0;
        /* 缓存大小 */
        unsigned char buff[1000] = {0};

        /* 接收数据 */
        m_serialBase->receive(buff);

        /* 数据解码 */
        size_t length = buff[1];
        if (buff[0] == 0xA0) {
            // ROS_WARN("Received navigation data,length: %d", length);
            // 导航数据 
            decode(&buff[2], length, f_data);
        } else if(buff[0] == 0xA1) {
            // ROS_WARN("Received RS data,length: %d", length);
            // 裁判系统数据
            UART_RSDataDecode(&buff[2], length);
        }
    }
}

