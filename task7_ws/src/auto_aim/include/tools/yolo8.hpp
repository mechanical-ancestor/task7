#ifndef YOLO8_HPP
#define YOLO8_HPP

#include <openvino/openvino.hpp>
#include "opencv2/opencv.hpp"
#include <iostream>
#include<vector>
#include<cmath>
#include<algorithm>
#include"opencv2/dnn.hpp"
#include"tools/Armor.hpp"

namespace tools{
  class YOLO8{
   std::string model_path;  //xml模型路径

   float conf_threshold;     //置信度过滤阈值

   float iou_shreshold;         //NMS极大值抑制阈值

   ov::InferRequest infer_request;         //推理请求对象

   const int model_w=640;     //定义YOLO检测输入图像的宽高为640*640
   const int model_h=640;     
   
   public:
   YOLO8(const std::string &model_path,float conf_threashold,float iou_threshold);

   ov::InferRequest Preprocces(std::string model_path);  //创建官方预处理模型

   ov::Tensor letterbox(cv::Mat image);   //图像预处理，将图像压缩为640*640并转为张量

   ov::Tensor infer(ov::Tensor &pre_image);  // 推理函数

   std::vector<Armor> Postprocess(const ov::Tensor &output_tensor,int &img_h, int &img_w);  //后处理函数

   static void drawContours(cv::Mat &image,const std::vector<Armor> &armors_List);  //检测框画框函数

   static void drawContours(cv::Mat &image,const Armor &armor);    //预测框画框函数

   std::vector<Armor> solver(const cv::Mat &image);  //yolo8主函数，输入图像，输出装甲板坐标和置信度

  };
}

#endif // YOLO8_HPP