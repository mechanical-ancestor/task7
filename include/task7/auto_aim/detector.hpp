#pragma once

#include <array>
#include <cmath>
#include <opencv2/core.hpp>
#include <vector>

#include "task7/config/config.hpp"

namespace task7::auto_aim {

struct LightBar {
    cv::RotatedRect rect;
    cv::Point2f center;
    cv::Point2f top;
    cv::Point2f bottom;
    float length;
    float width;
    float angle_deg;
    EnemyColor color;

    explicit LightBar(const cv::RotatedRect& r, EnemyColor detected_color)
        : rect(r),
          center(r.center),
          length(std::max(r.size.width, r.size.height)),
          width(std::min(r.size.width, r.size.height)),
          angle_deg(r.angle),
          color(detected_color) {
        if (r.size.width > r.size.height) {
            angle_deg = r.angle + 90.0f;
        }
        const float cos_a = std::cos(angle_deg * CV_PI / 180.0f);
        const float sin_a = std::sin(angle_deg * CV_PI / 180.0f);
        const float half_len = length * 0.5f;
        top = cv::Point2f(center.x - sin_a * half_len, center.y - cos_a * half_len);
        bottom = cv::Point2f(center.x + sin_a * half_len, center.y + cos_a * half_len);
    }
};

struct ArmorTarget {
    ArmorSize size{ArmorSize::kSmall};
    cv::Point2f center;
    std::array<cv::Point2f, 4> image_corners;
    cv::Vec3d rvec;
    cv::Vec3d tvec;
    cv::Point3f position_camera;
    float score{0.0f};

    ArmorTarget(const LightBar& left, const LightBar& right) {
        image_corners = {left.top, right.top, right.bottom, left.bottom};
        center = (left.center + right.center) * 0.5f;
    }
};

class Detector {
   public:
    explicit Detector(config::DetectionSettings settings);

    std::vector<ArmorTarget> detect(const cv::Mat& frame);
    void drawDebug(cv::Mat& frame) const;
    const cv::Mat& binaryImage() const;

   private:
    cv::Mat preprocess(const cv::Mat& frame) const;
    std::vector<LightBar> findLights(const cv::Mat& frame, const cv::Mat& binary) const;
    std::vector<ArmorTarget> pairLights(const std::vector<LightBar>& lights) const;
    bool isLight(const LightBar& light, double contour_area) const;
    EnemyColor estimateColor(const cv::Mat& frame, const cv::RotatedRect& rect) const;
    bool containsInterferingLight(
        const LightBar& left,
        const LightBar& right,
        const std::vector<LightBar>& lights) const;

    config::DetectionSettings settings_;
    mutable cv::Mat last_binary_;
    mutable std::vector<LightBar> last_lights_;
    mutable std::vector<ArmorTarget> last_armors_;
};

}  // namespace task7::auto_aim
