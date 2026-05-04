#include"tools/solverPnP.hpp"

namespace tools{
  solverPnP::solverPnP(){};

  std::vector<cv::Point3f> solverPnP::object_Points(const double &width,const double &height){
    return {
        cv::Point3f(-width/2,-height/2,0.0f),     //左上角
        cv::Point3f(width/2,-height/2,0.0f),      //右上角
        cv::Point3f(width/2,height/2,0.0f),       //右下角
        cv::Point3f(-width/2,height/2,0.0f)       //左下角
    };
  }

  std::vector<cv::Mat> solverPnP::solvePnp(std::vector<cv::Point3f> &object_Points,
                                                   Armor &armor,
                                                   cv::Mat &camera_matrix,
                                                   cv::Mat &dist_coeffs){
    std::vector<cv::Mat> result;
    cv::Mat rvec,tvec;
    //cv::solvePnP(object_Points,armor.Predict_Points,camera_matrix,dist_coeffs,rvec,tvec);
    cv::solvePnP(object_Points,armor.Predict_Points,camera_matrix,dist_coeffs,rvec,tvec,false,cv::SOLVEPNP_IPPE);
    result.push_back(rvec);
    result.push_back(tvec);
    return result;
  }
}