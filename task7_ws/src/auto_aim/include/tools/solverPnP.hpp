#ifndef SOLVERPNP_HPP
#define SOLVERPNP_HPP

#include "opencv2/opencv.hpp"
//#include "yolo8.hpp"
#include <vector>
#include "Eigen/Dense"
#include "tools/Armor.hpp"

namespace tools{
    class solverPnP{
        public:
        solverPnP();

        static std::vector<cv::Point3f> object_Points(const double &width,const double &height);  //生成装甲板的三维坐标

        static std::vector<cv::Mat> solvePnp(std::vector<cv::Point3f> &object_Points,
                                                   Armor &armor,
                                                   cv::Mat &camera_matrix,
                                                   cv::Mat &dist_coeffs
                                                );
    };
}

#endif // SOLVERPNP_HPP