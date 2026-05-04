#ifndef SERIAL_HPP
#define SERIAL_HPP

#include <iostream>
#include"Eigen/Dense"
#include<fcntl.h>
#include<unistd.h>
#include<termios.h>
#include<string>
#include"opencv2/opencv.hpp"
#include<sstream>

namespace Serial{
  class serial{
   private:
   int baudrate;
   int fd=-1;
   std::string _file;
   struct termios options;
   public:
      serial(const std::string &_file,int baudrate);
      ~serial();
      //函数重载，支持直接传入字符串或Eigen向量
      void write(const std::vector<cv::Mat> &data);  //支持直接传入旋转向量和平移向量组成的二维向量
      void write(const cv::Mat &rvec,const cv::Mat &tvec);
      void write(const cv::Point2f &centry);
      void write();
  };
}

#endif
