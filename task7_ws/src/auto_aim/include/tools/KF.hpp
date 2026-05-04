#ifndef KF_HPP
#define KF_HPP

#include"opencv2/opencv.hpp"
#include"Eigen/Dense"
#include"tools/Armor.hpp"

namespace tools{ 
  class KMFilter{

   double dt;

   Eigen::Matrix<double,4,1> X;                            //状态向量

   Eigen::Matrix4d P=Eigen::Matrix4d::Identity()*0.01;     //先验估计协方差

   Eigen::Matrix4d F;                                      //状态转移方程

   Eigen::Matrix4d Q=Eigen::Matrix4d::Identity()*0.01;     //估计噪声
  
   Eigen::Matrix<double,2,4> H;                            //观测矩阵

   Eigen::Matrix2d R=Eigen::Matrix2d::Identity()*0.01;     //观察噪声

   public:

     KMFilter();
     KMFilter(double dt);
     KMFilter(const Eigen::Matrix<double,4,1> &X0,const Eigen::Matrix2d &R0,
     const Eigen::Matrix4d &F0,const Eigen::Matrix4d &Q0,Eigen::Matrix4d &P0);

     cv::Point2f predict();                          //预测，得出先验估计和R
     void predict_P();                                
     void update(cv::Point2f &centry);               //更新数据，更新后验估计
 
  };
}


#endif