#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "task7/auto_aim/detector.hpp"
#include "task7/auto_aim/tracker.hpp"
#include "task7/auto_aim/solver.hpp"
#include "task7/calibration/calibrator.hpp"
#include "task7/calibration/capture.hpp"
#include "task7/config/config.hpp"
#include "task7/io/serial.hpp"
#include "task7/io/camera.hpp"

namespace {

struct CliOptions {
    std::string command{"autoaim"};
    std::string config_path{"config/app.yaml"};
    int camera_override{-1};
    bool disable_serial{false};
    bool print_help{false};
};

void printUsage() {
    std::cout
        << "Usage:\n"
        << "  task7_main [autoaim|capture|calibrate] [--config path] [--camera id] [--no-serial]\n";
}

bool parseArgs(int argc, char** argv, CliOptions& options, std::string& error) {
    for (int index = 1; index < argc; ++index) {
        const std::string arg = argv[index];
        if (arg == "autoaim" || arg == "capture" || arg == "calibrate") {
            options.command = arg;
            continue;
        }
        if (arg == "--config" && index + 1 < argc) {
            options.config_path = argv[++index];
            continue;
        }
        if (arg == "--camera" && index + 1 < argc) {
            options.camera_override = std::stoi(argv[++index]);
            continue;
        }
        if (arg == "--no-serial") {
            options.disable_serial = true;
            continue;
        }
        if (arg == "--help" || arg == "-h") {
            options.print_help = true;
            return true;
        }
        error = "Unknown argument: " + arg;
        return false;
    }
    return true;
}

std::string formatPredictionPayload(
    const task7::auto_aim::ArmorTarget* measurement,
    const cv::Point3f& predicted_position) {
    std::ostringstream oss;
    oss << "pred," << predicted_position.x << ',' << predicted_position.y << ',' << predicted_position.z;
    if (measurement != nullptr) {
        oss << ",meas," << measurement->position_camera.x << ',' << measurement->position_camera.y << ','
            << measurement->position_camera.z;
    }
    oss << '\n';
    return oss.str();
}

int runAutoAim(task7::config::AppSettings settings) {
    namespace fs = std::filesystem;

    task7::io::UsbCamera camera(settings.camera);
    if (!camera.open()) {
        std::cerr << "[main] failed to open camera\n";
        return 1;
    }

    task7::auto_aim::Detector detector(settings.detection);
    task7::auto_aim::PnpSolver solver(settings.solver);
    std::string calibration_error;
    if (!solver.loadCalibration(&calibration_error)) {
        std::cerr << "[main] " << calibration_error << '\n';
        return 1;
    }

    task7::auto_aim::ArmorTracker tracker(settings.tracker);
    task7::io::SerialPort serial(settings.serial);
    if (settings.serial.enabled && !serial.open()) {
        std::cerr << "[main] serial open failed, continuing without serial output\n";
    }

    std::ofstream record_file;
    if (!settings.runtime.record_output_path.empty()) {
        const fs::path record_path(settings.runtime.record_output_path);
        if (!record_path.parent_path().empty()) {
            fs::create_directories(record_path.parent_path());
        }
        record_file.open(record_path, std::ios::out | std::ios::trunc);
        if (record_file.is_open()) {
            record_file << "pred_x,pred_y,pred_z,meas_x,meas_y,meas_z\n";
        }
    }

    cv::namedWindow("task7_autoaim", cv::WINDOW_NORMAL);
    if (settings.runtime.draw_binary) {
        cv::namedWindow("task7_binary", cv::WINDOW_NORMAL);
    }

    while (true) {
        cv::Mat frame;
        if (!camera.read(frame) || frame.empty()) {
            continue;
        }

        std::vector<task7::auto_aim::ArmorTarget> armors = detector.detect(frame);
        int solved_count = 0;
        task7::auto_aim::ArmorTarget* best_armor = nullptr;
        float best_center_distance = std::numeric_limits<float>::max();

        for (auto& armor : armors) {
            if (!solver.solve(armor)) {
                continue;
            }

            ++solved_count;
            const float center_distance = solver.distanceToImageCenter(armor.center);
            if (center_distance < best_center_distance) {
                best_center_distance = center_distance;
                best_armor = &armor;
            }
        }

        const auto now = task7::auto_aim::ArmorTracker::Clock::now();
        if (best_armor != nullptr) {
            tracker.update(best_armor->position_camera, now);
        } else {
            tracker.markLostIfExpired(now);
        }

        detector.drawDebug(frame);

        bool has_prediction = tracker.hasTrack();
        cv::Point3f predicted_position{};
        if (has_prediction) {
            predicted_position = tracker.predictFuture(settings.tracker.prediction_lead_seconds);
        }

        if (best_armor != nullptr && has_prediction) {
            const std::vector<cv::Point2f> predicted_corners =
                solver.projectArmor(best_armor->size, best_armor->rvec, cv::Vec3d(
                    predicted_position.x,
                    predicted_position.y,
                    predicted_position.z));

            if (predicted_corners.size() == 4) {
                for (size_t index = 0; index < predicted_corners.size(); ++index) {
                    cv::line(
                        frame,
                        predicted_corners[index],
                        predicted_corners[(index + 1) % predicted_corners.size()],
                        cv::Scalar(0, 255, 255),
                        2);
                }
            }
        } else if (has_prediction) {
            const std::vector<cv::Point2f> predicted_center = solver.projectPoint(predicted_position);
            if (!predicted_center.empty()) {
                cv::circle(frame, predicted_center.front(), 6, cv::Scalar(0, 255, 255), 2);
            }
        }

        std::ostringstream status;
        status << "detected=" << armors.size() << " solved=" << solved_count;
        if (best_armor != nullptr) {
            status << " z=" << best_armor->position_camera.z << "m";
        }
        cv::putText(
            frame,
            status.str(),
            cv::Point(20, 30),
            cv::FONT_HERSHEY_SIMPLEX,
            0.8,
            cv::Scalar(0, 255, 255),
            2);
        cv::imshow("task7_autoaim", frame);

        if (settings.runtime.draw_binary && !detector.binaryImage().empty()) {
            cv::imshow("task7_binary", detector.binaryImage());
        }

        if (has_prediction) {
            const std::string payload = formatPredictionPayload(best_armor, predicted_position);
            if (serial.isOpen()) {
                serial.write(payload);
            }
            if (record_file.is_open()) {
                record_file << predicted_position.x << ',' << predicted_position.y << ',' << predicted_position.z;
                if (best_armor != nullptr) {
                    record_file << ',' << best_armor->position_camera.x << ',' << best_armor->position_camera.y << ','
                                << best_armor->position_camera.z;
                } else {
                    record_file << ",,,";
                }
                record_file << '\n';
            }
        }

        const int key = cv::waitKey(1);
        if (key == 'q' || key == 27) {
            break;
        }
    }

    camera.release();
    serial.close();
    cv::destroyAllWindows();
    return 0;
}

int runCalibrate(const task7::config::AppSettings& settings) {
    task7::calibration::CalibrationResult result;
    std::string error;
    if (!task7::calibration::calibrateFromDirectory(settings.calibration, result, &error)) {
        std::cerr << "[calibrate] " << error << '\n';
        return 1;
    }
    if (!task7::calibration::writeCalibrationFile(settings.calibration, result, &error)) {
        std::cerr << "[calibrate] " << error << '\n';
        return 1;
    }

    std::cout << "[calibrate] rms=" << result.rms
              << " mean_reprojection_error=" << result.mean_reprojection_error << '\n';
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    CliOptions options;
    std::string error;
    if (!parseArgs(argc, argv, options, error)) {
        std::cerr << error << '\n';
        printUsage();
        return 1;
    }
    if (options.print_help) {
        printUsage();
        return 0;
    }

    task7::config::AppSettings settings;
    if (!task7::config::loadAppSettings(options.config_path, settings, &error)) {
        std::cerr << error << '\n';
        return 1;
    }

    if (options.camera_override >= 0) {
        settings.camera.device_id = options.camera_override;
    }
    if (options.disable_serial) {
        settings.serial.enabled = false;
    }

    if (options.command == "capture") {
        return task7::calibration::runChessboardCapture(settings.camera, settings.calibration);
    }
    if (options.command == "calibrate") {
        return runCalibrate(settings);
    }
    return runAutoAim(settings);
}
