#pragma once

#include <opencv2/core.hpp>

#include <string>

namespace task7::auto_aim {

enum class EnemyColor { kRed, kBlue };
enum class ArmorSize { kSmall, kLarge };

inline std::string toString(EnemyColor color) {
    return color == EnemyColor::kRed ? "red" : "blue";
}

inline std::string toString(ArmorSize size) {
    return size == ArmorSize::kSmall ? "small" : "large";
}

}  // namespace task7::auto_aim

namespace task7::config {

struct CameraSettings {
    int device_id{0};
    int width{1280};
    int height{720};
    int fps{60};
    std::string backend{"auto"};
};

struct CalibrationSettings {
    int board_cols{9};
    int board_rows{6};
    float square_size{0.025f};
    std::string image_directory{"data/calibration_images"};
    std::string output_path{"config/camera.yaml"};
};

struct DetectionSettings {
    auto_aim::EnemyColor enemy_color{auto_aim::EnemyColor::kBlue};
    int binary_threshold{150};
    int color_threshold{45};
    int min_contour_area{20};
    cv::Size morph_kernel{3, 3};
    float light_min_ratio{2.5f};
    float light_max_ratio{20.0f};
    float light_max_angle_deg{40.0f};
    float light_max_angle_diff_deg{12.0f};
    float light_min_area{15.0f};
    float armor_min_small_center_dist{1.0f};
    float armor_max_small_center_dist{4.0f};
    float armor_min_large_center_dist{3.6f};
    float armor_max_large_center_dist{6.6f};
    float armor_max_vertical_misalignment_ratio{0.7f};
    float armor_max_length_ratio{1.6f};
};

struct SolverSettings {
    std::string calibration_path{"config/camera.yaml"};
    float small_armor_width{0.135f};
    float small_armor_height{0.055f};
    float large_armor_width{0.230f};
    float large_armor_height{0.055f};
};

struct TrackerSettings {
    double prediction_lead_seconds{0.03};
    double lost_reset_seconds{0.40};
    double process_noise_position{1e-3};
    double process_noise_velocity{5e-3};
    double measurement_noise{1e-2};
};

struct SerialSettings {
    bool enabled{false};
    std::string port{"/dev/ttyUSB0"};
    int baudrate{115200};
    int timeout_ms{20};
};

struct RuntimeSettings {
    bool show_debug{true};
    bool draw_binary{false};
    std::string record_output_path{"output/predictions.csv"};
};

struct AppSettings {
    CameraSettings camera;
    CalibrationSettings calibration;
    DetectionSettings detection;
    SolverSettings solver;
    TrackerSettings tracker;
    SerialSettings serial;
    RuntimeSettings runtime;
};

bool loadAppSettings(const std::string& config_path, AppSettings& settings, std::string* error);

}  // namespace task7::config
