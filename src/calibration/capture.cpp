#include "task7/calibration/capture.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "task7/io/camera.hpp"

namespace task7::calibration {

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

    const cv::Size board_size(calibration_settings.board_cols, calibration_settings.board_rows);
    int saved_count = 0;
    cv::namedWindow("task7_capture", cv::WINDOW_NORMAL);

    while (true) {
        cv::Mat frame;
        if (!camera.read(frame) || frame.empty()) {
            continue;
        }

        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> corners;
        const bool found = cv::findChessboardCorners(
            gray,
            board_size,
            corners,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

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
            "Press c to save, q to quit",
            cv::Point(20, 30),
            cv::FONT_HERSHEY_SIMPLEX,
            0.8,
            cv::Scalar(0, 255, 255),
            2);
        cv::imshow("task7_capture", frame);

        const int key = cv::waitKey(1);
        if (key == 'q' || key == 27) {
            break;
        }
        if (key == 'c' && found) {
            std::ostringstream oss;
            oss << calibration_settings.image_directory << "/img_" << std::setw(3) << std::setfill('0') << saved_count
                << ".png";
            cv::imwrite(oss.str(), frame);
            std::cout << "[capture] saved " << oss.str() << '\n';
            ++saved_count;
        }
    }

    camera.release();
    cv::destroyWindow("task7_capture");
    return 0;
}

}  // namespace task7::calibration
