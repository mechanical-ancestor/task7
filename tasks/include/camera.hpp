#ifndef CAMERA_HPP
#define CAMERA_HPP



#include <opencv2/opencv.hpp> 
#include <yaml-cpp/yaml.h>  
#include <string>
#include "MvCameraControl.h"


class  RealtimeCamera{
public:
    // 构造函数、析构函数声明
    RealtimeCamera();
    ~RealtimeCamera();
    

    // 成员函数
     bool open();
     bool start();
     cv::Mat getFrame();
     void stop();
     bool loadCameraCalibParams(const std::string& camera_sources_path);


     //参数
    cv::Mat camera_matrix;   // 相机内参矩阵
    cv::Mat dist_coeffs;     // 相机畸变系数
    void* handle_;          // 相机句柄 海康相机句柄
    bool is_collect;        // 采集状态


};

#endif //CAMERA_HPP