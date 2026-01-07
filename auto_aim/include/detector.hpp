#ifndef AUTO_AIM_DETECTOR_HPP
#define AUTO_AIM_DETECTOR_HPP

#include <vector>
#include <opencv2/core.hpp>
#include "type.hpp"

namespace auto_aim {
class Detector
{
public:
// 检测参数
struct Params {
    // 预处理参数
    struct PreprocessParam {
        int binary_threshold;      // 二值化阈值
        EnemyColor enemy_color = EnemyColor::UNKNOWN;
    } prepc_params;
    // 灯条参数
    struct LightParam {
    float min_light_ratio;     // 最小灯条长宽比
        float max_light_ratio;     // 最大灯条长宽比
        float max_light_angle;     // 最大灯条倾斜角
    } light_params;
    // 判断装甲板参数
    struct ArmorParam {
        float min_small_center_dist;     // 最小 小装甲板 中心距
        float max_small_center_dist;     // 最大 小装甲板 中心距
        float min_large_center_dist;     // 最小 大装甲板 中心距
        float max_large_center_dist;     // 最大 大装甲板 中心距
        float max_angle_diff;            // 最大灯条角度差

    } armor_params;

    bool loadParams(const std::string& path);
};

public:
    explicit Detector(const Params& param);

    // 主检测接口
    std::vector<Armor> detect(const cv::Mat& src);

    // 绘制调试信息
    void drawDebug(cv::Mat& img, cv::Scalar colors) const;

private:
    // 内部处理流程
    cv::Mat preprocess(const cv::Mat& src);
    std::vector<Light> findLights(const cv::Mat& bin);
    std::vector<Armor> matchLights(const std::vector<Light>& lights);

    // 判定函数
    bool isLight(const Light& light) const;
    ArmorType isArmor(const Light& l1, const Light& l2) const;

private:
    Params param_;
    
    cv::Mat processed_img_;
    std::vector<Light> detected_lights_;
    std::vector<Armor> detected_armors_;
};

} // namespace auto_aim
#endif // AUTO_AIM_DETECTOR_HPP
