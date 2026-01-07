#ifndef AUTO_AIM_SOLVER_HPP
#define AUTO_AIM_SOLVER_HPP

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "type.hpp"

namespace auto_aim {

class Solver {
public:
    // ------ 传入相机标定文件路径 -------//
    explicit Solver(const std::string& calib_path);
    // ------ PnP解算 -----------------//
    bool solve(Armor& armor) const;
    // ------ 绘制解算结果 ------------ //
    void printSolverDebug(cv::Mat& frame, const std::vector<Armor>& armors);
    // ------ 得到参数的接口 ---------- // 
    const cv::Mat getCameraMatrix() const {return camera_matrix_;}
    const cv::Mat getDistCoeffs() const {return dist_coeffs_;}

private:
    // 相机参数
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

    // 装甲板 3D 点
    std::vector<cv::Point3f> small_armor_pts_;
    std::vector<cv::Point3f> big_armor_pts_;
};

} // namespace auto_aim

#endif // AUTO_AIM_SOLVER_HPP
