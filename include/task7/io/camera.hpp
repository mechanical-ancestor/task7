#pragma once

#include <opencv2/videoio.hpp>

#include <string>

#include "task7/config/config.hpp"

namespace task7::io {

class UsbCamera {
   public:
    explicit UsbCamera(config::CameraSettings settings);

    bool open();
    bool read(cv::Mat& frame);
    bool isOpened() const;
    void release();
    std::string name() const;

   private:
    int resolveBackend(const std::string& backend_name);

    config::CameraSettings settings_;
    cv::VideoCapture capture_;
};

}  // namespace task7::io
