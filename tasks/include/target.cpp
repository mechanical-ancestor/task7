#include <iostream>
#include <opencv2/opencv.hpp> 
#include "target.hpp"

//构造函数初始化
target::target(double dt) {
    //  初始化
    dt_=dt;
    
    // 手动初始化卡尔曼核心变量（替换原OpenCV KalmanFilter）
    x_ = cv::Mat::zeros(4, 1, CV_64F);     // 状态向量初始化为0
    P_ = cv::Mat::eye(4, 4, CV_64F) * 1.0; // 误差协方差矩阵 P（和原1.0一致）
    Q_ = cv::Mat::eye(4, 4, CV_64F) * 0.01;// 噪声协方差 Q(过程) 越大越信任观测值
    R_ = cv::Mat::eye(2, 2, CV_64F) * 0.1; // 噪声协方差 R(观测) 越大越信任预测值

    //  初始状态（和原逻辑一致，仅换载体）
    x_.at<double>(0) = 0.0; // x
    x_.at<double>(1) = 0.0; // vx
    x_.at<double>(2) = 0.0; // y
    x_.at<double>(3) = 0.0; // vy
}

cv::Mat target::predict() {
    // 手动实现预测
    //  状态转移矩阵 F
    cv::Mat F = cv::Mat::eye(4, 4, CV_64F);        
    F.at<double>(0, 1) = dt_;
    F.at<double>(2, 3) = dt_;
    /*[1, dt, 0,  0]  // x = x_prev + vx_prev × dt
      [0,  1, 0,  0]  // vx = vx_prev
      [0,  0, 1, dt]  // y = y_prev + vy_prev × dt
      [0,  0, 0,  1]  // vy = vy_prev）*/

    //  先验估计
    x_ = F * x_;             
    P_ = F * P_ * F.t() + Q_;

    //  返回预测状态（转换为float，和原接口返回类型一致）
    cv::Mat predicted_state(4, 1, CV_32F);
    predicted_state.at<float>(0) = static_cast<float>(x_.at<double>(0));
    predicted_state.at<float>(1) = static_cast<float>(x_.at<double>(1));
    predicted_state.at<float>(2) = static_cast<float>(x_.at<double>(2));
    predicted_state.at<float>(3) = static_cast<float>(x_.at<double>(3));
    return predicted_state;
}

void target::correct(const cv::Point2f& measure_point) {
    // 校正
    // 构造观测向量（保留原逻辑，仅扩展为double类型）
    cv::Mat z = (cv::Mat_<double>(2, 1) << measure_point.x, measure_point.y);

    //  观测矩阵 H
    cv::Mat H = cv::Mat::zeros(2, 4, CV_64F);
    H.at<double>(0, 0) = 1; // 仅观测x
    H.at<double>(1, 2) = 1; // 仅观测y

    //  计算残差
    cv::Mat y = z - H * x_;
    //  残差协方差
    cv::Mat S = H * P_ * H.t() + R_;
    //  卡尔曼增益
    cv::Mat K = P_ * H.t() * S.inv(); 

    //  更新状态和协方差
    x_ = x_ + K * y;
    cv::Mat I = cv::Mat::eye(4, 4, CV_64F);
    P_ = (I - K * H) * P_;
}