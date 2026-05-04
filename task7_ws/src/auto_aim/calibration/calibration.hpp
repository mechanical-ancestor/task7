#ifndef CALIBRATION_HPP
#define CALIBRATION_HPP

#include"opencv2/opencv.hpp"

namespace calibration{
class calibration{
    std::vector<std::vector<cv::Point3f>> obj_points;  //棋盘角点
    std::vector<std::vector<cv::Point2f>> img_points;  //图像坐标系角点
    
    //世界坐标系
    int image_height;
    int image_width;
    cv::Size boardsize;
    float squareSize=25.0f;   //方块边长
    std::vector<cv::Point3f> objp;   //每个棋盘方块的角点
    std::vector<std::string> filenames;

    //标定产生的内参
    cv::Mat camera_matrix;
    cv::Mat dist_matrix;
    std::vector<cv::Mat> rvec,tvec;

   public:
    //构造函数
    calibration(int image_height,int image_width,int board_width,int board_height,float squreSize);


    bool calibration_(std::string &dataSet_path);

    void save_result(std::string save_path="result");
   };

}

#endif