#pragma once

#include <opencv2/core.hpp>

#include <string>

#include "task7/config/config.hpp"

namespace task7::calibration {

struct CalibrationResult {
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    double rms{0.0};
    double mean_reprojection_error{0.0};
    cv::Size image_size;
};

bool calibrateFromDirectory(
    const config::CalibrationSettings& settings,
    CalibrationResult& result,
    std::string* error = nullptr);

bool writeCalibrationFile(
    const config::CalibrationSettings& settings,
    const CalibrationResult& result,
    std::string* error = nullptr);

}  // namespace task7::calibration
