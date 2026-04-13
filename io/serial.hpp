#ifndef SERIAL_HPP
#define SERIAL_HPP

// 必要头文件（字符串传输+串口配置+输入输出）
#include <termios.h>   // 串口参数配置
#include <string>      // 字符串操作
#include <iostream>    // 控制台打印（cout/cerr）
#include <csignal>
// 声明全局停止标志（用于退出接收循环，和主程序联动）
extern volatile sig_atomic_t g_stop_flag;

class serial {
private:
    // 类成员：串口文件描述符（替代全局变量，更安全）
    int serial_fd = -1;

    // 辅助函数声明（字符串打包/解析，私有化，外部无需调用）
    std::string pack_to_string(float x, float y);
    bool unpack_from_string(const std::string& recv_str, float& x, float& y);

public:
    // 串口核心函数声明（逻辑一致）
    bool serial_begin(const char* port, speed_t baud, int& fd); 
    bool send_predict(float x, float y);
    bool recv_predict();
    void serial_close();  
};

#endif // SERIAL_HPP