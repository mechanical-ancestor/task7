#pragma once

#include <opencv2/core.hpp>

#include <string>
#include <vector>

#include "task7/auto_aim/detector.hpp"
#include "task7/config/config.hpp"

namespace task7::auto_aim {

class PnpSolver {
   public:
    explicit PnpSolver(config::SolverSettings settings);

    bool loadCalibration(std::string* error = nullptr);
    bool solve(ArmorTarget& armor) const;
    bool ready() const;

    float distanceToImageCenter(const cv::Point2f& image_point) const;

    std::vector<cv::Point2f> projectArmor(
        ArmorSize size,
        const cv::Vec3d& rvec,
        const cv::Vec3d& tvec) const;

    std::vector<cv::Point2f> projectPoint(const cv::Point3f& position) const;

    const cv::Mat& cameraMatrix() const;
    const cv::Mat& distCoeffs() const;

   private:
    cv::Point2f imageCenter() const;
    std::vector<cv::Point3f> armorModel(ArmorSize size) const;

    config::SolverSettings settings_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
};

}  // namespace task7::auto_aim
