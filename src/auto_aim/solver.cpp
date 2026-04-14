#include "task7/auto_aim/solver.hpp"

#include <opencv2/calib3d.hpp>

#include <limits>
#include <utility>

namespace task7::auto_aim {

PnpSolver::PnpSolver(config::SolverSettings settings)
    : settings_(std::move(settings)) {}

bool PnpSolver::loadCalibration(std::string* error) {
    cv::FileStorage fs(settings_.calibration_path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        if (error != nullptr) {
            *error = "Failed to open calibration file: " + settings_.calibration_path;
        }
        return false;
    }

    fs["camera_matrix"] >> camera_matrix_;
    fs["dist_coeffs"] >> dist_coeffs_;

    if (camera_matrix_.empty() || dist_coeffs_.empty()) {
        if (error != nullptr) {
            *error = "Calibration file is missing camera_matrix or dist_coeffs";
        }
        return false;
    }

    camera_matrix_ = camera_matrix_.clone();
    dist_coeffs_ = dist_coeffs_.clone();
    return true;
}

bool PnpSolver::solve(ArmorTarget& armor) const {
    if (!ready()) {
        return false;
    }

    const std::vector<cv::Point3f> object_points = armorModel(armor.size);
    if (object_points.size() != 4) {
        return false;
    }

    const std::vector<cv::Point2f> image_points(armor.image_corners.begin(), armor.image_corners.end());
    cv::Vec3d rvec;
    cv::Vec3d tvec;
    const bool ok = cv::solvePnP(
        object_points,
        image_points,
        camera_matrix_,
        dist_coeffs_,
        rvec,
        tvec,
        false,
        cv::SOLVEPNP_IPPE);

    if (!ok) {
        return false;
    }

    armor.rvec = rvec;
    armor.tvec = tvec;
    armor.position_camera = cv::Point3f(
        static_cast<float>(tvec[0]),
        static_cast<float>(tvec[1]),
        static_cast<float>(tvec[2]));
    return true;
}

bool PnpSolver::ready() const {
    return !camera_matrix_.empty() && !dist_coeffs_.empty();
}

float PnpSolver::distanceToImageCenter(const cv::Point2f& image_point) const {
    if (!ready()) {
        return std::numeric_limits<float>::max();
    }
    return cv::norm(image_point - imageCenter());
}

std::vector<cv::Point2f> PnpSolver::projectArmor(
    ArmorSize size,
    const cv::Vec3d& rvec,
    const cv::Vec3d& tvec) const {
    std::vector<cv::Point2f> image_points;
    if (!ready()) {
        return image_points;
    }

    const std::vector<cv::Point3f> object_points = armorModel(size);
    cv::projectPoints(object_points, rvec, tvec, camera_matrix_, dist_coeffs_, image_points);
    return image_points;
}

std::vector<cv::Point2f> PnpSolver::projectPoint(const cv::Point3f& position) const {
    std::vector<cv::Point2f> image_points;
    if (!ready()) {
        return image_points;
    }

    const std::vector<cv::Point3f> object_points = {position};
    cv::projectPoints(
        object_points,
        cv::Vec3d::zeros(),
        cv::Vec3d::zeros(),
        camera_matrix_,
        dist_coeffs_,
        image_points);
    return image_points;
}

const cv::Mat& PnpSolver::cameraMatrix() const {
    return camera_matrix_;
}

const cv::Mat& PnpSolver::distCoeffs() const {
    return dist_coeffs_;
}

cv::Point2f PnpSolver::imageCenter() const {
    if (!ready()) {
        return {};
    }
    return cv::Point2f(
        static_cast<float>(camera_matrix_.at<double>(0, 2)),
        static_cast<float>(camera_matrix_.at<double>(1, 2)));
}

std::vector<cv::Point3f> PnpSolver::armorModel(ArmorSize size) const {
    float width = settings_.small_armor_width;
    float height = settings_.small_armor_height;
    if (size == ArmorSize::kLarge) {
        width = settings_.large_armor_width;
        height = settings_.large_armor_height;
    }

    const float half_width = width * 0.5f;
    const float half_height = height * 0.5f;

    return {
        {0.0f, -half_width, half_height},
        {0.0f, half_width, half_height},
        {0.0f, half_width, -half_height},
        {0.0f, -half_width, -half_height},
    };
}

}  // namespace task7::auto_aim
