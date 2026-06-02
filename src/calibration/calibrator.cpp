#include "task7/calibration/calibrator.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <vector>

namespace task7::calibration {

namespace {

bool hasSupportedImageExtension(const std::filesystem::path& path) {
    const std::string ext = path.extension().string();
    return ext == ".png" || ext == ".jpg" || ext == ".jpeg" || ext == ".bmp";
}

std::vector<cv::Point3f> chessboardModel(int cols, int rows, float square_size) {
    std::vector<cv::Point3f> model;
    model.reserve(static_cast<size_t>(cols * rows));
    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            model.emplace_back(col * square_size, row * square_size, 0.0f);
        }
    }
    return model;
}

}  // namespace

bool calibrateFromDirectory(
    const config::CalibrationSettings& settings,
    CalibrationResult& result,
    std::string* error) {
    namespace fs = std::filesystem;

    if (!fs::exists(settings.image_directory)) {
        if (error != nullptr) {
            *error = "Calibration image directory does not exist: " + settings.image_directory;
        }
        return false;
    }

    std::vector<fs::path> images;
    for (const auto& entry : fs::directory_iterator(settings.image_directory)) {
        if (entry.is_regular_file() && hasSupportedImageExtension(entry.path())) {
            images.push_back(entry.path());
        }
    }
    std::sort(images.begin(), images.end());

    if (images.size() < 8) {
        if (error != nullptr) {
            *error = "Need at least 8 chessboard images for calibration";
        }
        return false;
    }

    const cv::Size board_size(settings.board_cols - 1, settings.board_rows - 1);
    const std::vector<cv::Point3f> model = chessboardModel(
        settings.board_cols - 1,
        settings.board_rows - 1,
        settings.square_size);

    std::vector<std::vector<cv::Point2f>> image_points;
    std::vector<std::vector<cv::Point3f>> object_points;
    cv::Size image_size;

    for (const auto& image_path : images) {
        cv::Mat image = cv::imread(image_path.string(), cv::IMREAD_COLOR);
        if (image.empty()) {
            continue;
        }

        image_size = image.size();
        cv::Mat gray;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(
            gray,
            board_size,
            corners,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

#if CV_VERSION_MAJOR >= 4
        if (!found) {
            found = cv::findChessboardCornersSB(
                gray,
                board_size,
                corners,
                cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_EXHAUSTIVE | cv::CALIB_CB_ACCURACY);
        }
#endif

        if (!found) {
            std::cout << "[calibrate] rejected, no corners: " << image_path.filename().string() << '\n';
            continue;
        }
        if (corners.size() != model.size()) {
            std::cout << "[calibrate] rejected, corner count mismatch: " << image_path.filename().string() << '\n';
            continue;
        }

        cv::cornerSubPix(
            gray,
            corners,
            cv::Size(11, 11),
            cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));

        image_points.push_back(corners);
        object_points.push_back(model);
        std::cout << "[calibrate] accepted: " << image_path.filename().string() << '\n';
    }

    if (image_points.size() < 8) {
        if (error != nullptr) {
            *error = "Not enough valid chessboard detections were found";
        }
        return false;
    }

    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;
    result.rms = cv::calibrateCamera(
        object_points,
        image_points,
        image_size,
        result.camera_matrix,
        result.dist_coeffs,
        rvecs,
        tvecs);
    result.image_size = image_size;

    double total_error = 0.0;
    size_t total_points = 0;
    for (size_t index = 0; index < object_points.size(); ++index) {
        std::vector<cv::Point2f> reprojected;
        cv::projectPoints(
            object_points[index],
            rvecs[index],
            tvecs[index],
            result.camera_matrix,
            result.dist_coeffs,
            reprojected);

        const double error_value = cv::norm(image_points[index], reprojected, cv::NORM_L2);
        total_error += error_value * error_value;
        total_points += object_points[index].size();
    }
    result.mean_reprojection_error = std::sqrt(total_error / std::max<size_t>(1, total_points));
    return true;
}

bool writeCalibrationFile(
    const config::CalibrationSettings& settings,
    const CalibrationResult& result,
    std::string* error) {
    namespace fs = std::filesystem;

    const fs::path output_path(settings.output_path);
    if (!output_path.parent_path().empty()) {
        fs::create_directories(output_path.parent_path());
    }

    cv::FileStorage fs_writer(settings.output_path, cv::FileStorage::WRITE);
    if (!fs_writer.isOpened()) {
        if (error != nullptr) {
            *error = "Failed to write calibration file: " + settings.output_path;
        }
        return false;
    }

    fs_writer << "camera_matrix" << result.camera_matrix;
    fs_writer << "dist_coeffs" << result.dist_coeffs;
    fs_writer << "rms" << result.rms;
    fs_writer << "mean_reprojection_error" << result.mean_reprojection_error;
    fs_writer << "image_width" << result.image_size.width;
    fs_writer << "image_height" << result.image_size.height;
    fs_writer << "board_cols" << settings.board_cols;
    fs_writer << "board_rows" << settings.board_rows;
    fs_writer << "square_size" << settings.square_size;
    return true;
}

}  // namespace task7::calibration
