
#ifndef YOLOV8_HPP
#define YOLOV8_HPP

//  必须的依赖头文件
#include <openvino/openvino.hpp>  // OpenVINO核心
#include <opencv2/opencv.hpp>     
#include <yaml-cpp/yaml.h>        // YAML配置解析
#include <list>                   // std::list容器
#include "armor.hpp"              // 装甲板数据




//namespace aim{ 
class yolov8 {
public:
    
    // int class_num_;  // 装甲板类别数（分类用，不分类则删除）这里我没分类

    // 其他成员变量
    ov::Core core_;
    ov::CompiledModel compiled_model_;
    std::string model_path_;
    std::string device_;
    double min_confidence_;
    cv::Rect roi_cut;
    cv::Point2f restore_;
    bool use_roi;
    std::string detector_;

//关键

    // 构造函数
    yolov8(const std::string& yolov8_yaml_path);

    // 检测函数
    std::list<Armor> detect(const cv::Mat& raw_img, int frame_count);

    // 结果解析函数（私有也可以，看你需求，这里放public方便一点吧）
    std::list<Armor> parse(double scale, cv::Mat& output, const cv::Mat& bgr_img, int frame_count);

    // 绘制检测框的函数
    void draw_detections(const cv::Mat& img, const std::list<Armor>& armors, int frame_count) const;


};
//}

#endif