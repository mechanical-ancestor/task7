#include "task7/config/config.hpp"

#include <opencv2/core.hpp>

#include <algorithm>
#include <cctype>

namespace task7::config {

namespace {

template <typename T>
void readIfPresent(const cv::FileNode& node, const char* key, T& value) {
    const cv::FileNode child = node[key];
    if (!child.empty()) {
        child >> value;
    }
}

std::string toLowerCopy(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return value;
}

auto_aim::EnemyColor parseEnemyColor(const std::string& value) {
    return toLowerCopy(value) == "red" ? auto_aim::EnemyColor::kRed : auto_aim::EnemyColor::kBlue;
}

}  // namespace

bool loadAppSettings(const std::string& config_path, AppSettings& settings, std::string* error) {
    cv::FileStorage fs(config_path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        if (error != nullptr) {
            *error = "Failed to open config file: " + config_path;
        }
        return false;
    }

    const cv::FileNode camera = fs["camera"];
    if (!camera.empty()) {
        readIfPresent(camera, "device_id", settings.camera.device_id);
        readIfPresent(camera, "width", settings.camera.width);
        readIfPresent(camera, "height", settings.camera.height);
        readIfPresent(camera, "fps", settings.camera.fps);
        readIfPresent(camera, "backend", settings.camera.backend);
    }

    const cv::FileNode calibration = fs["calibration"];
    if (!calibration.empty()) {
        readIfPresent(calibration, "board_cols", settings.calibration.board_cols);
        readIfPresent(calibration, "board_rows", settings.calibration.board_rows);
        readIfPresent(calibration, "square_size", settings.calibration.square_size);
        readIfPresent(calibration, "image_directory", settings.calibration.image_directory);
        readIfPresent(calibration, "output_path", settings.calibration.output_path);
    }

    const cv::FileNode detection = fs["detection"];
    if (!detection.empty()) {
        std::string enemy_color = auto_aim::toString(settings.detection.enemy_color);
        readIfPresent(detection, "enemy_color", enemy_color);
        settings.detection.enemy_color = parseEnemyColor(enemy_color);
        readIfPresent(detection, "binary_threshold", settings.detection.binary_threshold);
        readIfPresent(detection, "color_threshold", settings.detection.color_threshold);
        readIfPresent(detection, "min_contour_area", settings.detection.min_contour_area);
        readIfPresent(detection, "morph_width", settings.detection.morph_kernel.width);
        readIfPresent(detection, "morph_height", settings.detection.morph_kernel.height);
        readIfPresent(detection, "light_min_ratio", settings.detection.light_min_ratio);
        readIfPresent(detection, "light_max_ratio", settings.detection.light_max_ratio);
        readIfPresent(detection, "light_max_angle_deg", settings.detection.light_max_angle_deg);
        readIfPresent(detection, "light_max_angle_diff_deg", settings.detection.light_max_angle_diff_deg);
        readIfPresent(detection, "light_min_area", settings.detection.light_min_area);
        readIfPresent(detection, "armor_min_small_center_dist", settings.detection.armor_min_small_center_dist);
        readIfPresent(detection, "armor_max_small_center_dist", settings.detection.armor_max_small_center_dist);
        readIfPresent(detection, "armor_min_large_center_dist", settings.detection.armor_min_large_center_dist);
        readIfPresent(detection, "armor_max_large_center_dist", settings.detection.armor_max_large_center_dist);
        readIfPresent(
            detection,
            "armor_max_vertical_misalignment_ratio",
            settings.detection.armor_max_vertical_misalignment_ratio);
        readIfPresent(detection, "armor_max_length_ratio", settings.detection.armor_max_length_ratio);
    }

    const cv::FileNode solver = fs["solver"];
    if (!solver.empty()) {
        readIfPresent(solver, "calibration_path", settings.solver.calibration_path);
        readIfPresent(solver, "small_armor_width", settings.solver.small_armor_width);
        readIfPresent(solver, "small_armor_height", settings.solver.small_armor_height);
        readIfPresent(solver, "large_armor_width", settings.solver.large_armor_width);
        readIfPresent(solver, "large_armor_height", settings.solver.large_armor_height);
    }

    const cv::FileNode tracker = fs["tracker"];
    if (!tracker.empty()) {
        readIfPresent(tracker, "prediction_lead_seconds", settings.tracker.prediction_lead_seconds);
        readIfPresent(tracker, "lost_reset_seconds", settings.tracker.lost_reset_seconds);
        readIfPresent(tracker, "process_noise_position", settings.tracker.process_noise_position);
        readIfPresent(tracker, "process_noise_velocity", settings.tracker.process_noise_velocity);
        readIfPresent(tracker, "measurement_noise", settings.tracker.measurement_noise);
    }

    const cv::FileNode serial = fs["serial"];
    if (!serial.empty()) {
        int enabled = settings.serial.enabled ? 1 : 0;
        readIfPresent(serial, "enabled", enabled);
        settings.serial.enabled = enabled != 0;
        readIfPresent(serial, "port", settings.serial.port);
        readIfPresent(serial, "baudrate", settings.serial.baudrate);
        readIfPresent(serial, "timeout_ms", settings.serial.timeout_ms);
    }

    const cv::FileNode runtime = fs["runtime"];
    if (!runtime.empty()) {
        int show_debug = settings.runtime.show_debug ? 1 : 0;
        int draw_binary = settings.runtime.draw_binary ? 1 : 0;
        readIfPresent(runtime, "show_debug", show_debug);
        readIfPresent(runtime, "draw_binary", draw_binary);
        settings.runtime.show_debug = show_debug != 0;
        settings.runtime.draw_binary = draw_binary != 0;
        readIfPresent(runtime, "record_output_path", settings.runtime.record_output_path);
    }

    return true;
}

}  // namespace task7::config
