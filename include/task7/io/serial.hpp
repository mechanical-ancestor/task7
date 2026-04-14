#pragma once

#include <string>

#include "task7/config/config.hpp"

namespace task7::io {

class SerialPort {
   public:
    explicit SerialPort(config::SerialSettings settings);
    ~SerialPort();

    bool open();
    void close();
    bool isOpen() const;
    bool write(const std::string& payload) const;

   private:
    config::SerialSettings settings_;
#ifdef _WIN32
    void* handle_{nullptr};
#else
    int fd_{-1};
#endif
};

}  // namespace task7::io
