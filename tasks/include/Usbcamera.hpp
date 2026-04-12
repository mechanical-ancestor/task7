#ifndef USB_CAMERA_HPP
#define USB_CAMERA_HPP

#include <opencv2/opencv.hpp>

class UsbCamera
{
public:
    cv::VideoCapture cap_;       // 摄像头句柄
    bool is_opened_;             // 是否打开

    // 相机内参（用于去畸变）
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;

    UsbCamera();
    ~UsbCamera();

    // 打开USB摄像头（默认0）
    bool open(int camera_id = 0);

    // 关闭摄像头
    void close();

    // 获取一帧图像
    cv::Mat getFrame();

    // 加载相机内参
    bool loadCalibParams(const std::string& yaml_path);

    // 保存图片
    bool saveImage(const cv::Mat& frame, const std::string& filename);

    // 判断是否打开
    bool isOpened() const;
};

#endif