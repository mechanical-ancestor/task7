#include "task7/io/camera.hpp"

#include <iostream>
#include <utility>

namespace task7::io {

UsbCamera::UsbCamera(config::CameraSettings settings)
    : settings_(std::move(settings)) {}

bool UsbCamera::open() {
    const int backend = resolveBackend(settings_.backend);
    if (!capture_.open(settings_.device_id, backend)) {
        std::cerr << "[camera] failed to open device " << settings_.device_id << '\n';
        return false;
    }

    capture_.set(cv::CAP_PROP_FRAME_WIDTH, settings_.width);
    capture_.set(cv::CAP_PROP_FRAME_HEIGHT, settings_.height);
    capture_.set(cv::CAP_PROP_FPS, settings_.fps);
    return true;
}

bool UsbCamera::read(cv::Mat& frame) {
    return capture_.read(frame);
}

bool UsbCamera::isOpened() const {
    return capture_.isOpened();
}

void UsbCamera::release() {
    capture_.release();
}

std::string UsbCamera::name() const {
    return "UsbCamera";
}

int UsbCamera::resolveBackend(const std::string& backend_name) {
    if (backend_name == "auto") {
#ifdef _WIN32
        return cv::CAP_DSHOW;
#else
        return cv::CAP_ANY;
#endif
    }
    if (backend_name == "dshow") {
        return cv::CAP_DSHOW;
    }
    if (backend_name == "msmf") {
        return cv::CAP_MSMF;
    }
    if (backend_name == "v4l2") {
        return cv::CAP_V4L2;
    }
    return cv::CAP_ANY;
}

}  // namespace task7::io
