#include "task7/calibration/capture.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <exception>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "task7/io/camera.hpp"

namespace task7::calibration {

namespace {

std::string imagePathForIndex(const std::string& directory, int index) {
    std::ostringstream oss;
    oss << directory << "/img_" << std::setw(3) << std::setfill('0') << index << ".png";
    return oss.str();
}

int nextImageIndex(const std::string& directory) {
    int next_index = 0;
    if (!std::filesystem::exists(directory)) {
        return next_index;
    }

    for (const auto& entry : std::filesystem::directory_iterator(directory)) {
        if (!entry.is_regular_file()) {
            continue;
        }
        const std::string stem = entry.path().stem().string();
        if (stem.rfind("img_", 0) != 0) {
            continue;
        }
        try {
            next_index = std::max(next_index, std::stoi(stem.substr(4)) + 1);
        } catch (const std::exception&) {
            continue;
        }
    }
    return next_index;
}

}  // namespace

int runChessboardCapture(
    const config::CameraSettings& camera_settings,
    const config::CalibrationSettings& calibration_settings) {
    namespace fs = std::filesystem;

    fs::create_directories(calibration_settings.image_directory);

    io::UsbCamera camera(camera_settings);
    if (!camera.open()) {
        std::cerr << "[capture] failed to open camera\n";
        return 1;
    }

    const cv::Size board_size(calibration_settings.board_cols - 1, calibration_settings.board_rows - 1);
    int saved_count = nextImageIndex(calibration_settings.image_directory);
    cv::namedWindow("task7_capture", cv::WINDOW_NORMAL);

    while (true) {
        cv::Mat frame;
        if (!camera.read(frame) || frame.empty()) {
            continue;
        }
        const cv::Mat raw_frame = frame.clone();

        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

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

        if (found) {
            cv::cornerSubPix(
                gray,
                corners,
                cv::Size(11, 11),
                cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
            cv::drawChessboardCorners(frame, board_size, corners, found);
        }

        cv::putText(
            frame,
            found ? "CHESSBOARD DETECTED - press c to save, q to quit"
                  : "No chessboard - press c to save, q to quit",
            cv::Point(20, 30),
            cv::FONT_HERSHEY_SIMPLEX,
            0.7,
            found ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 165, 255),
            2);
        cv::putText(
            frame,
            "Saved: " + std::to_string(saved_count),
            cv::Point(20, 60),
            cv::FONT_HERSHEY_SIMPLEX,
            0.6,
            cv::Scalar(255, 255, 255),
            1);
        cv::imshow("task7_capture", frame);

        const int key = cv::waitKey(1);
        if (key == 'q' || key == 27) {
            break;
        }
        if (key == 'c') {
            if (found) {
                const std::string output_path = imagePathForIndex(calibration_settings.image_directory, saved_count);
                cv::imwrite(output_path, raw_frame);
                std::cout << "[capture] saved " << output_path << '\n';
                ++saved_count;
            } else {
                std::cout << "[capture] no chessboard detected, not saved\n";
            }
        }
    }

    camera.release();
    cv::destroyWindow("task7_capture");
    return 0;
}

}  // namespace task7::calibration
