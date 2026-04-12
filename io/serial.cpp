#include "serial.hpp"
#include <fcntl.h>    // 串口打开（open）、文件控制
#include <unistd.h>   // 串口关闭（close）、延时（usleep）
#include <cstring>    // 内存操作（memset）、字符串函数（strcmp）
#include <cerrno>     // 错误码（errno、EAGAIN）
#include <sstream>    // 字符串拼接/分割（stringstream）
#include <stdexcept>  // 异常处理



// ===================== 串口初始化 =====================
bool serial::serial_begin(const char* port, speed_t baud, int& fd) {
    // 1. 打开串口设备
    fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    serial_fd = fd;  // 赋值给类成员
    if (fd == -1) {
        perror("Open serial port failed");
        return false;
    }

    // 2. 配置串口参数
    struct termios opt;
    memset(&opt, 0, sizeof(opt));
    if (tcgetattr(fd, &opt) != 0) {
        perror("Get serial attributes failed");
        close(fd);
        serial_fd = -1;
        return false;
    }

    // 3. 设置波特率
    cfsetispeed(&opt, baud);
    cfsetospeed(&opt, baud);

    // 4. 配置核心通信参数（8N1：8位数据位、无校验、1位停止位）
    opt.c_cflag |= CLOCAL | CREAD | CS8;        // 本地连接、允许接收、8位数据位
    opt.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS); // 关闭校验、1位停止位、关闭硬件流控
    opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // 关闭规范模式、回显、信号响应
    opt.c_oflag &= ~OPOST;                      // 关闭输出处理（原始模式）
    opt.c_cc[VTIME] = 0;                        // 非阻塞读取（无需等待）
    opt.c_cc[VMIN] = 0;

    // 5. 生效配置
    if (tcsetattr(fd, TCSANOW, &opt) != 0) {
        perror("Set serial attributes failed");
        close(fd);
        serial_fd = -1;
        return false;
    }

    // 6. 清空串口缓存
    tcflush(fd, TCIOFLUSH);
    std::cout << "Serial port " << port << " initialized successfully!" << std::endl;
    return true;
}

// ===================== 辅助：打包为字符串 =====================
std::string serial::pack_to_string(float x, float y) {
    // 格式："PRED:X,Y\n" （\n作为行结束符，方便接收端解析）
    std::stringstream ss;
    ss << "PRED:" << x << "," << y << "\n";
    return ss.str();
}

// ===================== 发送预测坐标 =====================
bool serial::send_predict(float x, float y) {
    // 1. 校验串口是否打开
    if (serial_fd == -1) {
        std::cerr << "Error: Serial port not opened!" << std::endl;
        return false;
    }

    // 2. 打包为字符串
    std::string send_str = pack_to_string(x, y);

    // 3. 发送字符串
    ssize_t write_bytes = write(serial_fd, send_str.c_str(), send_str.size());
    if (write_bytes == static_cast<ssize_t>(send_str.size())) {
        std::cout << "Send success: PRED(" << x << ", " << y << ")" << std::endl;
        // 缓存最新发送的坐标
        // this->predict_x = x;
        // this->predict_y = y;
        return true;
    } else if (write_bytes == -1) {
        perror("Serial send failed");
        return false;
    } else {
        std::cerr << "Warning: Incomplete send! Sent " << write_bytes 
                  << " bytes, need " << send_str.size() << " bytes" << std::endl;
        return false;
    }
}

// ===================== 辅助：解析字符串 =====================
bool serial::unpack_from_string(const std::string& recv_str, float& x, float& y) {
    // 1. 校验前缀（必须以PRED:开头）
    const std::string prefix = "PRED:";
    if (recv_str.find(prefix) != 0) {
        std::cerr << "Invalid prefix: " << recv_str << std::endl;
        return false;
    }

    // 2. 截取数据部分（去掉PRED:）
    std::string data_part = recv_str.substr(prefix.size());
    
    // 3. 按逗号分割x和y
    size_t comma_pos = data_part.find(',');
    if (comma_pos == std::string::npos) {
        std::cerr << "Missing comma: " << data_part << std::endl;
        return false;
    }

    // 4. 转换为浮点数
    try {
        x = std::stof(data_part.substr(0, comma_pos));
        y = std::stof(data_part.substr(comma_pos + 1));
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Convert failed: " << e.what() << " | Data: " << data_part << std::endl;
        return false;
    }
}

// ===================== 接收预测坐标 =====================
bool serial::recv_predict() {
    // 1. 校验串口是否打开
    if (serial_fd == -1) {
        std::cerr << "Error: Serial port not opened!" << std::endl;
        return false;
    }

    std::cout << "Start receiving serial data... (Press Ctrl+C to exit)" << std::endl;
    char buf[128] = {0};          // 临时接收缓冲区
    std::string recv_buf;         // 拼接完整行的缓冲区

    // 2. 循环接收（直到全局停止标志置位）
    while (!g_stop_flag) {
        // 非阻塞读取串口数据
        ssize_t read_bytes = read(serial_fd, buf, sizeof(buf) - 1);
        
        if (read_bytes > 0) {
            // 3. 拼接接收到的字符
            buf[read_bytes] = '\0';  // 字符串结束符
            recv_buf += buf;

            // 4. 按换行符分割完整行（处理粘包）
            size_t newline_pos = recv_buf.find('\n');
            while (newline_pos != std::string::npos) {
                // 提取一行完整数据
                std::string one_line = recv_buf.substr(0, newline_pos);
                // 移除已解析的部分，保留剩余字符
                recv_buf = recv_buf.substr(newline_pos + 1);

                // 5. 解析该行数据
                float x, y;
                if (unpack_from_string(one_line, x, y)) {
                    std::cout << "=================\nReceived: PRED(" << x << ", " << y << ")\n";
                }

                // 继续检查剩余缓冲区是否有换行符
                newline_pos = recv_buf.find('\n');
            }
        } else if (read_bytes == -1) {
            // 排除非阻塞无数据的正常情况（EAGAIN）
            if (errno != EAGAIN) {
                perror("Serial read failed");
                usleep(10000);  // 延时避免刷屏
            }
        }

        // 降低CPU占用
        usleep(1000);
    }

    std::cout << "Receive loop exited." << std::endl;
    return true;
}

// ===================== 关闭串口 =====================
void serial::serial_close() {
    if (serial_fd != -1) {
        close(serial_fd);
        serial_fd = -1;
        std::cout << "Serial port closed successfully." << std::endl;
    }
}