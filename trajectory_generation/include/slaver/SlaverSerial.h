#pragma once
#include <dirent.h>
#include <boost/thread/mutex.hpp>
#include <string>
#include "SerialBase.h"


class SlaverSerial {
public:
    union float2uchar {
        float f;
        unsigned char ch[4];
    };

    /* 枚举类型： 串口数据类型*/
    enum DATA_TYPE {
        NONE,
        ENEMY_COLOR,
        ARRIVE_GOAL,
    };

    SlaverSerial(const std::string str, int baud_rate);
    ~SlaverSerial();
    unsigned char decode(unsigned char* buff, size_t& received_length,
                std::vector<float>& num);
    bool decode_num(unsigned char buff, float& num);
    void send(std::vector<float> num_f = std::vector<float>(), std::vector<unsigned char> num_c = std::vector<unsigned char>());
    void receive(std::vector<float>& f_data);

private:
    SerialBase* m_serialBase;                ///串口底层
    unsigned char FIRST_ONE;                 ///协议
    unsigned char FIRST_TWO;                 ///协议
    unsigned char LAST_TWO;                  ///协议
    unsigned char LAST_ONE;                  ///协议
    unsigned char MAX_RECEIVE_FLOAT_LENGTH;  ///最大接收数据数

    // lock
    boost::mutex m_send_mutex;
};
