#include "tasks/include/Usbcamera.hpp"
#include <iostream>

UsbCamera::UsbCamera() : is_opened_(false) {}

UsbCamera::~UsbCamera()
{
    close();
}

// 🔥 修复这里：加上 V4L2 + 格式 + 分辨率，解决 cwbad conversion
bool UsbCamera::open(int camera_id)
{
     
    // 强制使用 Linux 原生摄像头驱动（必须加）
   cap_.open(camera_id, cv::CAP_V4L2);

    // 强制设置格式和尺寸（解决格式报错）
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));

    is_opened_ = cap_.isOpened();
    return is_opened_;
}

void UsbCamera::close()
{
    if (is_opened_) {
        cap_.release();
        is_opened_ = false;
    }
}

cv::Mat UsbCamera::getFrame()
{
    if (!is_opened_) {
        return cv::Mat();
    }

    cv::Mat frame;
    cap_ >> frame;
    if (frame.empty()) {
        return cv::Mat();
    }

    // 如果加载了内参，自动去畸变
    if (!camera_matrix.empty() && !dist_coeffs.empty()) {
        cv::Mat undistorted_frame;
        cv::undistort(frame, undistorted_frame, camera_matrix, dist_coeffs);
        return undistorted_frame;
    }

    return frame;
}

bool UsbCamera::loadCalibParams(const std::string& yaml_path)
{
    cv::FileStorage fs(yaml_path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        return false;
    }

    fs["camera_matrix"] >> camera_matrix;
    fs["dist_coeffs"] >> dist_coeffs;
    fs.release();

    return !camera_matrix.empty() && !dist_coeffs.empty();
}

bool UsbCamera::saveImage(const cv::Mat& frame, const std::string& filename)
{
    if (frame.empty()) return false;
    return cv::imwrite(filename, frame);
}

bool UsbCamera::isOpened() const
{
    return is_opened_;
}