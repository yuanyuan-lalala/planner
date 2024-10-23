//
// Created by zkz on 2021/10/31.
//
#include "SerialBase.h"

/**
 * 构造函数
 * @param str COM 口编号
 * @param baud_rate 波特率
 * @return
 */
SerialBase::SerialBase(std::string str, int baud_rate) {
    m_io = new boost::asio::io_service;
    m_port =
        new boost::asio::serial_port(*m_io, str.c_str());  /// str相当于 COM 口
    m_port->set_option(
        boost::asio::serial_port::baud_rate(baud_rate));  ///设置通信波特率
    m_port->set_option(boost::asio::serial_port::flow_control());  ///流量监控
    m_port->set_option(boost::asio::serial_port::parity());     ///奇偶校验
    m_port->set_option(boost::asio::serial_port::stop_bits());  ///停止位
    m_port->set_option(
        boost::asio::serial_port::character_size(8));  ///字符大小
}

SerialBase::~SerialBase() {
    delete m_io;
    delete m_port;
}
/**
 * 发送接口
 * @param ch 待发送字符串
 * @param length 发送字符串长度
 * @return
 */
void SerialBase::send(unsigned char *ch, size_t length) {
    boost::asio::write(*m_port, boost::asio::buffer(ch, length));
}
/**
 * 接收接口
 * @param buff: data buffer for receive
 * @return 0xAB：数据开头   0xA0/0xA1: 数据类型  第三个字节：有效数据总长度（不计前三个数据识别字节）
 */
void SerialBase::receive(unsigned char *buff) {
    boost::system::error_code err;

    unsigned char temp_c = 0;
    boost::asio::read(*m_port, boost::asio::buffer(&temp_c, 1));
    if (temp_c == 0xAB) {
        boost::asio::read(*m_port, boost::asio::buffer(&temp_c, 1));
        if (temp_c == 0xA0) {
            boost::asio::read(*m_port, boost::asio::buffer(&temp_c, 1));
            buff[0] = 0xA0;
            buff[1] = temp_c;
            boost::asio::read(*m_port, boost::asio::buffer(&buff[2], temp_c));
        } else if (temp_c == 0xA1) {
            boost::asio::read(*m_port, boost::asio::buffer(&temp_c, 1));
            buff[0] = 0xA1;
            buff[1] = temp_c;
            boost::asio::read(*m_port, boost::asio::buffer(&buff[2], temp_c));
        }
    }

    // if (temp_c == 0xA5) {
    //     boost::asio::read(*m_port, boost::asio::buffer(&temp_c, 1));
    //     buff[0] = 0xA5;
    //     buff[1] = temp_c;
    //     boost::asio::read(*m_port, boost::asio::buffer(&buff[2], 1));
    // }
}
