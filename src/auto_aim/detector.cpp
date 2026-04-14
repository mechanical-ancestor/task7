#include "task7/auto_aim/detector.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>

namespace task7::auto_aim {

namespace {

float safeRatio(float numerator, float denominator) {
    return numerator / std::max(denominator, 1e-3f);
}

}  // namespace

Detector::Detector(config::DetectionSettings settings)
    : settings_(std::move(settings)) {}

std::vector<ArmorTarget> Detector::detect(const cv::Mat& frame) {
    if (frame.empty()) {
        last_binary_.release();
        last_lights_.clear();
        last_armors_.clear();
        return {};
    }

    last_binary_ = preprocess(frame);
    last_lights_ = findLights(frame, last_binary_);
    last_armors_ = pairLights(last_lights_);
    return last_armors_;
}

void Detector::drawDebug(cv::Mat& frame) const {
    for (const auto& light : last_lights_) {
        cv::line(frame, light.top, light.bottom, cv::Scalar(255, 255, 0), 2);
        cv::circle(frame, light.center, 3, cv::Scalar(255, 255, 255), -1);
    }

    for (const auto& armor : last_armors_) {
        const std::array<cv::Point, 4> polygon = {
            cv::Point(cvRound(armor.image_corners[0].x), cvRound(armor.image_corners[0].y)),
            cv::Point(cvRound(armor.image_corners[1].x), cvRound(armor.image_corners[1].y)),
            cv::Point(cvRound(armor.image_corners[2].x), cvRound(armor.image_corners[2].y)),
            cv::Point(cvRound(armor.image_corners[3].x), cvRound(armor.image_corners[3].y)),
        };

        for (size_t index = 0; index < polygon.size(); ++index) {
            cv::line(frame, polygon[index], polygon[(index + 1) % polygon.size()], cv::Scalar(0, 255, 0), 2);
        }

        cv::circle(frame, armor.center, 4, cv::Scalar(0, 0, 255), -1);
        cv::putText(
            frame,
            toString(armor.size),
            armor.center + cv::Point2f(6.0f, -6.0f),
            cv::FONT_HERSHEY_SIMPLEX,
            0.5,
            cv::Scalar(0, 255, 255),
            1);
    }
}

const cv::Mat& Detector::binaryImage() const {
    return last_binary_;
}

cv::Mat Detector::preprocess(const cv::Mat& frame) const {
    std::vector<cv::Mat> channels;
    cv::split(frame, channels);

    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    cv::Mat color_delta;
    if (settings_.enemy_color == EnemyColor::kRed) {
        cv::subtract(channels[2], channels[0], color_delta);
    } else {
        cv::subtract(channels[0], channels[2], color_delta);
    }

    cv::Mat color_mask;
    cv::Mat bright_mask;
    cv::threshold(color_delta, color_mask, settings_.color_threshold, 255, cv::THRESH_BINARY);
    cv::threshold(gray, bright_mask, settings_.binary_threshold, 255, cv::THRESH_BINARY);

    cv::Mat binary;
    cv::bitwise_and(color_mask, bright_mask, binary);

    const cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_RECT,
        cv::Size(std::max(settings_.morph_kernel.width, 1), std::max(settings_.morph_kernel.height, 1)));
    cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);
    return binary;
}

std::vector<LightBar> Detector::findLights(const cv::Mat& frame, const cv::Mat& binary) const {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<LightBar> lights;
    lights.reserve(contours.size());

    for (const auto& contour : contours) {
        const double contour_area = cv::contourArea(contour);
        if (contour_area < settings_.min_contour_area) {
            continue;
        }

        const cv::RotatedRect rect = cv::minAreaRect(contour);
        const EnemyColor detected_color = estimateColor(frame, rect);
        const LightBar light(rect, detected_color);
        if (!isLight(light, contour_area)) {
            continue;
        }
        if (light.color != settings_.enemy_color) {
            continue;
        }
        lights.push_back(light);
    }

    std::sort(lights.begin(), lights.end(), [](const LightBar& lhs, const LightBar& rhs) {
        return lhs.center.x < rhs.center.x;
    });

    return lights;
}

std::vector<ArmorTarget> Detector::pairLights(const std::vector<LightBar>& lights) const {
    std::vector<ArmorTarget> armors;

    for (size_t left_index = 0; left_index < lights.size(); ++left_index) {
        for (size_t right_index = left_index + 1; right_index < lights.size(); ++right_index) {
            const LightBar& left = lights[left_index];
            const LightBar& right = lights[right_index];

            if (left.color != right.color) {
                continue;
            }

            const float average_length = std::max((left.length + right.length) * 0.5f, 1e-3f);
            const float length_ratio =
                safeRatio(std::max(left.length, right.length), std::min(left.length, right.length));
            const float angle_diff = std::abs(left.angle_deg - right.angle_deg);
            const float center_distance_ratio = safeRatio(cv::norm(left.center - right.center), average_length);
            const float vertical_offset_ratio = safeRatio(std::abs(left.center.y - right.center.y), average_length);

            if (length_ratio > settings_.armor_max_length_ratio) {
                continue;
            }
            if (angle_diff > settings_.light_max_angle_diff_deg) {
                continue;
            }
            if (vertical_offset_ratio > settings_.armor_max_vertical_misalignment_ratio) {
                continue;
            }
            if (containsInterferingLight(left, right, lights)) {
                continue;
            }

            ArmorTarget armor(left, right);
            if (center_distance_ratio >= settings_.armor_min_small_center_dist &&
                center_distance_ratio <= settings_.armor_max_small_center_dist) {
                armor.size = ArmorSize::kSmall;
            } else if (
                center_distance_ratio >= settings_.armor_min_large_center_dist &&
                center_distance_ratio <= settings_.armor_max_large_center_dist) {
                armor.size = ArmorSize::kLarge;
            } else {
                continue;
            }

            armor.score = 1.0f / (1.0f + center_distance_ratio + vertical_offset_ratio + angle_diff * 0.1f);
            armors.push_back(armor);
        }
    }

    std::sort(armors.begin(), armors.end(), [](const ArmorTarget& lhs, const ArmorTarget& rhs) {
        return lhs.score > rhs.score;
    });
    return armors;
}

bool Detector::isLight(const LightBar& light, double contour_area) const {
    const float ratio = safeRatio(light.length, light.width);
    if (ratio < settings_.light_min_ratio || ratio > settings_.light_max_ratio) {
        return false;
    }
    if (light.angle_deg > settings_.light_max_angle_deg) {
        return false;
    }
    if (contour_area < settings_.light_min_area) {
        return false;
    }
    return true;
}

EnemyColor Detector::estimateColor(const cv::Mat& frame, const cv::RotatedRect& rect) const {
    cv::Rect roi = rect.boundingRect();
    roi &= cv::Rect(0, 0, frame.cols, frame.rows);
    if (roi.empty()) {
        return EnemyColor::kBlue;
    }

    const cv::Scalar mean_color = cv::mean(frame(roi));
    return mean_color[2] > mean_color[0] ? EnemyColor::kRed : EnemyColor::kBlue;
}

bool Detector::containsInterferingLight(
    const LightBar& left,
    const LightBar& right,
    const std::vector<LightBar>& lights) const {
    const std::array<cv::Point2f, 4> armor_points = {left.top, right.top, right.bottom, left.bottom};
    const cv::Rect armor_box = cv::boundingRect(armor_points);

    for (const auto& candidate : lights) {
        if (candidate.center == left.center || candidate.center == right.center) {
            continue;
        }
        if (armor_box.contains(candidate.center)) {
            return true;
        }
    }

    return false;
}

}  // namespace task7::auto_aim
