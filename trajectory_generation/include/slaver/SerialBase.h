#pragma once
#include <boost/asio.hpp>
#include <iostream>

class SerialBase {  ///串口通信的基础类
   public:
    SerialBase(std::string str, int baud_rate);
    ~SerialBase();
    void send(unsigned char* ch, size_t length);
    void receive(unsigned char* buff);

   private:
    boost::asio::io_service* m_io;             /// boost 的I/O服务类指针
    boost::asio::serial_port* m_port;          /// boost 串口通信的端口
    static const int MAX_BUFFER_LENGTH = 200;  ///最大缓冲区长度
};
