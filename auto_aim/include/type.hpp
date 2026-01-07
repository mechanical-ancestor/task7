#ifndef AUTO_AIM_TYPE_HPP
#define AUTO_AIM_TYPE_HPP

#include <vector>
#include <opencv2/core.hpp>

namespace auto_aim {

enum class EnemyColor {
    RED,
    BLUE,
    UNKNOWN
};

enum class ArmorType {
    SMALL,
    LARGE,
    UNKNOWN
};

// 灯条
struct Light {
    float angle;
    float length;
    float width;

    cv::RotatedRect rect;
    cv::Point2f center;
    cv::Point2f top;
    cv::Point2f bottom;

    EnemyColor color;
};

// 装甲板
struct Armor {
    /* -------- Detector 阶段 -------- */
    cv::Point2f center;
    Light left_light;
    Light right_light;
    ArmorType type = ArmorType::UNKNOWN;

    /* -------- Solver 阶段 -------- */
    std::vector<cv::Point2f> points;   // 4 个角点
    cv::Point3f position;              // PnP 解算结果,单位 m
};

} // namespace auto_aim

#endif // AUTO_AIM_TYPE_HPP
