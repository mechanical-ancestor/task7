#include "task7/io/serial.hpp"

#include <iostream>
#include <utility>

#ifdef _WIN32
#include <windows.h>
#else
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace task7::io {

SerialPort::SerialPort(config::SerialSettings settings)
    : settings_(std::move(settings)) {}

SerialPort::~SerialPort() {
    close();
}

bool SerialPort::open() {
#ifdef _WIN32
    std::string port_name = settings_.port;
    if (port_name.rfind("\\\\.\\", 0) != 0 && port_name.rfind("COM", 0) == 0 && port_name.size() > 4) {
        port_name = "\\\\.\\" + port_name;
    }

    HANDLE handle = CreateFileA(
        port_name.c_str(),
        GENERIC_READ | GENERIC_WRITE,
        0,
        nullptr,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL,
        nullptr);

    if (handle == INVALID_HANDLE_VALUE) {
        std::cerr << "[serial] failed to open " << settings_.port << '\n';
        return false;
    }

    DCB dcb{};
    dcb.DCBlength = sizeof(DCB);
    if (!GetCommState(handle, &dcb)) {
        CloseHandle(handle);
        std::cerr << "[serial] failed to query serial state\n";
        return false;
    }

    dcb.BaudRate = static_cast<DWORD>(settings_.baudrate);
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;

    if (!SetCommState(handle, &dcb)) {
        CloseHandle(handle);
        std::cerr << "[serial] failed to configure serial state\n";
        return false;
    }

    COMMTIMEOUTS timeouts{};
    timeouts.ReadIntervalTimeout = static_cast<DWORD>(settings_.timeout_ms);
    timeouts.ReadTotalTimeoutConstant = static_cast<DWORD>(settings_.timeout_ms);
    timeouts.WriteTotalTimeoutConstant = static_cast<DWORD>(settings_.timeout_ms);
    SetCommTimeouts(handle, &timeouts);

    handle_ = handle;
    return true;
#else
    fd_ = ::open(settings_.port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) {
        std::cerr << "[serial] failed to open " << settings_.port << '\n';
        return false;
    }

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
        close();
        std::cerr << "[serial] failed to query termios\n";
        return false;
    }

    speed_t speed = B115200;
    if (settings_.baudrate == 9600) {
        speed = B9600;
    } else if (settings_.baudrate == 921600) {
        speed = B921600;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_iflag = 0;
    tty.c_oflag = 0;
    tty.c_lflag = 0;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        close();
        std::cerr << "[serial] failed to apply termios settings\n";
        return false;
    }

    return true;
#endif
}

void SerialPort::close() {
#ifdef _WIN32
    if (handle_ != nullptr) {
        CloseHandle(static_cast<HANDLE>(handle_));
        handle_ = nullptr;
    }
#else
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
#endif
}

bool SerialPort::isOpen() const {
#ifdef _WIN32
    return handle_ != nullptr;
#else
    return fd_ >= 0;
#endif
}

bool SerialPort::write(const std::string& payload) const {
    if (!isOpen()) {
        return false;
    }

#ifdef _WIN32
    DWORD bytes_written = 0;
    return WriteFile(
        static_cast<HANDLE>(handle_),
        payload.data(),
        static_cast<DWORD>(payload.size()),
        &bytes_written,
        nullptr) == TRUE && bytes_written == payload.size();
#else
    return ::write(fd_, payload.data(), payload.size()) == static_cast<ssize_t>(payload.size());
#endif
}

}  // namespace task7::io
