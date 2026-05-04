#include"tools/KF.hpp"

namespace tools{
     KMFilter::KMFilter(double dt):dt(dt){
        F<<1,0,dt,0,    //初始化状态转移矩阵
           0,1,0,dt,
           0,0,1,0,
           0,0,0,1;

        H<<1,0,0,0,       //初始化观测矩阵
           0,1,0,0;

        X<<0,0,0,0;       //初始化状态向量
     };

     KMFilter::KMFilter(const Eigen::Matrix<double,4,1> &X0,const Eigen::Matrix2d &R0,
     const Eigen::Matrix4d &F0,const Eigen::Matrix4d &Q0,Eigen::Matrix4d &P0){
            this->X=X0;
            this->F=F0;
            this->Q=Q0;
            this->P=P0;
            this->R=R0;

            H<<1,0,0,0,      
               0,1,0,0;
       }

      cv::Point2f KMFilter::predict(){  //当有物体时返回预测坐标

       P=F*P*F.transpose()+Q;

       Eigen::MatrixXd result=F*X;      //状态转移方程

       return cv::Point2f(static_cast<float>(result(0)),static_cast<float>(result(1)));
       //return cv::Point2f(result(0),result(1));
    }

    void KMFilter::predict_P(){          //没有物体时就只是更新P

        P=F*P*F.transpose()+Q; 

    }

    void KMFilter::update(cv::Point2f &centry){             //更新后验估计，估计协方差

          Eigen::Matrix4d I=Eigen::Matrix4d::Identity();     //单位矩阵

          Eigen::Vector2d Z;                                 //实际观测值

          Z<<centry.x,centry.y;

          Eigen::MatrixXd K;                                  //卡尔曼增益

          K=P*H.transpose()*(H*P*H.transpose()+R).inverse();  //卡尔曼增益计算

          X=X+K*(Z-H*X);                                      //后验估计

          P=(I-K*H)*P;                                        //后验协方差更新

    }
}

