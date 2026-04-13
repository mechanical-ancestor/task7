
#ifndef ARMOR_HPP
#define ARMOR_HPP

#include <opencv2/opencv.hpp>

// 绘制装甲板所需的矩形参数
class Armor {
public:
    int x;          // 装甲板矩形左上角x坐标
    int y;          // 装甲板矩形左上角y坐标
    int width;      // 装甲板矩形宽度
    int height;     // 装甲板矩形高度
    float confidence; // 装甲板检测置信度
    cv::Rect box;   //yolov8.cpp有用到
    // 构造函数，直接初始化矩形参数
    Armor(int x_, int y_, int w_, int h_) 
      { x=x_; 
        y=y_; 
        width=w_; 
        height=h_;/*将值赋给成员*/
     }


        Armor(int/*保持与其他函数的格式一致防止报错*/, float conf, const cv::Rect& box)
        {
            confidence=conf;
             x=box.x; 
             y=box.y;
            width=box.width;
             height=box.height; 
        }

    // 带ROI偏移（如果 parse() 中 use_roi=true 的话）
    Armor(int /*作用同上*/, float conf, const cv::Rect& box, const cv::Point2f& restore_)
       {  confidence=conf; 
          x=box.x + restore_.x;  // 还原到原始图像坐标
          y=box.y + restore_.y;
          width=box.width; 
          height=box.height;   
       }

    };//!!不要忘记这个分号啊

#endif 