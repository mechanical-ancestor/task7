#ifndef PARAM_HPP_
#define PARAM_HPP_

#include<iostream>
#include"yaml-cpp/yaml.h"
#include"opencv2/opencv.hpp"
#include"Eigen/Dense"

namespace tools{
class param{
    public:
    //camera
     cv::Mat Camera_Matrix;
     cv::Mat distortion_coefficients;
    //yolo8
     std::string xml_path;
     double conf_threshold;
     double iou_threshold;
    //serial
     std::string _file;
     int baudrate;
    //kf
     double dt;
    std::string boot_path="..";
     param(){
        cv::FileStorage fs(boot_path+"/config/camera.yaml",cv::FileStorage::READ);
        fs["camera_matrix"] >> Camera_Matrix;
        fs["distortion_coefficients"] >> distortion_coefficients;

        YAML::Node data=YAML::LoadFile(boot_path+"/config/config.yaml");
        //yolo8
        xml_path=data["YOLO8"]["xml_path"].as<std::string>();
        conf_threshold=data["YOLO8"]["conf_threshold"].as<double>();
        iou_threshold=data["YOLO8"]["iou_threshold"].as<double>();
        //serial
        _file=data["serial"]["_file"].as<std::string>();
        baudrate=data["serial"]["baudrate"].as<int>();
        //kf
        dt=data["kf"]["dt"].as<double>();
   };
  };
}

#endif