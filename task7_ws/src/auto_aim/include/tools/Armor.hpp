#ifndef ARMOR_HPP
#define ARMOR_HPP

#include"opencv2/opencv.hpp"

namespace tools{

class Armor{
    public:
    cv::Rect box; //轮廓

    cv::Point2f centry;  //中心坐标

    float confidence; //置信度

    std::vector<cv::Point2f> Predict_Points;  //四个角点坐标，顺序为左上、右上、右下、左下,与pnp算法对应

    Armor(){};
    
    Armor(cv::Rect box,float confidence){
        this->box=box;
        this->confidence=confidence;
        this->centry=cv::Point2f(box.x+box.width/2.0f,box.y+box.height/2.0f);
    }

     void get_predict(const cv::Point2f &centry) {                   //根据卡尔曼滤波预测的中心坐标更新装甲板的四个角点坐标,原点在左上角
        Predict_Points.clear();
         this->Predict_Points.push_back(cv::Point2f(                 //左上
            std::max(centry.x-box.width/2.0f,0.0f),
            std::max(centry.y-box.height/2.0f,0.0f)
        ));  
         this->Predict_Points.push_back(cv::Point2f(                  //右上
            std::max(centry.x+box.width/2.0f,0.0f),
            std::max(centry.y-box.height/2.0f,0.0f)
        ));  
         this->Predict_Points.push_back(cv::Point2f(                  //右下
            std::max(centry.x+box.width/2.0f,0.0f),
            std::max(centry.y+box.height/2.0f,0.0f)
        ));  
         this->Predict_Points.push_back(cv::Point2f(                  //左下
            std::max(centry.x-box.width/2.0f,0.0f),
            std::max(centry.y+box.height/2.0f,0.0f)
        ));  
     }
 };
}

#endif // ARMOR_HPP