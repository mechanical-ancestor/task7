
#ifndef TARGET_HPP
#define TARGET_HPP

#include <opencv2/opencv.hpp>  // 引入 OpenCV 头文件，提供 cv::KalmanFilter 等
#include <iostream>            

// 封装成类
class target {
public:
    // 初始化卡尔曼滤波器的相关参数
    target(double dt);
                    
    
    // 预测
    cv::Mat predict();
    
    // 修正
    void correct(const cv::Point2f& detect_point);

private:
    double dt_;  // 采样时间间隔（相机采集频率的倒数 ，30 帧/秒则 dt=1/30）
    //cv::KalmanFilter kf_;  // 卡尔曼滤波器预测对象
    cv::Mat x_;          // 状态向量 (4x1, CV_64F)
    cv::Mat P_;          // 协方差矩阵 (4x4, CV_64F)
    cv::Mat Q_;          // 过程噪声 (4x4, CV_64F)
    cv::Mat R_;          // 观测噪声 (2x2, CV_64F)
    bool is_initialized_; // 可选
};

#endif