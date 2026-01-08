#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include "detector.hpp"

namespace auto_aim{
    Detector::Detector(const Params& param) : param_(param) {}

    // 主检测接口
    std::vector<Armor> Detector::detect(const cv::Mat& src) {
        if (src.empty()) return {};

        processed_img_ = preprocess(src);
        
        detected_lights_ = findLights(processed_img_);
        
        detected_armors_ = matchLights(detected_lights_);
        
        return detected_armors_;
    }

    // 图像预处理
    cv::Mat Detector::preprocess(const cv::Mat& src) {
        cv::Mat gray, bin, morph;
        
        cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);

        cv::threshold(gray, bin, param_.prepc_params.binary_threshold, 255, cv::THRESH_BINARY);
        cv::Mat kernel = 
            cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7,7));

        // 腐蚀
        cv::erode(bin, morph, kernel);
        // 膨胀
        cv::dilate(morph, morph, kernel);
        return morph;
    }

    // 寻找灯条
    std::vector<Light> Detector::findLights(const cv::Mat& bin){
        std::vector<Light> detected_lights;
        std::vector<std::vector<cv::Point>> contours; 
        cv::findContours(bin, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        for(const auto& contour : contours){
            cv::RotatedRect rect = cv::minAreaRect(contour);
            Light light;
            
            light.rect   = rect;
            light.center = rect.center;
            light.length = std::max(rect.size.width, rect.size.height);
            light.width  = std::min(rect.size.width, rect.size.height);
            light.angle  = rect.angle;

            if (!isLight(light))   continue;
            
            cv::Point2f pts[4]; // 灯条矩形的四个角点
            rect.points(pts);
            
            float max_dist = 0.0f;
            cv::Point2f p1, p2;
            for (int i = 0; i < 4; ++i) {
                for (int j = i + 1; j < 4; ++j) {
                    float dist = cv::norm(pts[i] - pts[j]);
                    if (dist > max_dist) {
                        max_dist = dist;
                        p1 = pts[i];
                        p2 = pts[j];
                    }
                }
            }
            // 统一 top 在上
            if (p1.y < p2.y) {
                light.top    = p1;
                light.bottom = p2;
            } else {
                light.top    = p2;
                light.bottom = p1;
            }
            detected_lights.push_back(light);   
        }
        return detected_lights;
    } 

    // 判定是否为灯条
    bool Detector::isLight(const Light& light) const{
        // 长宽比判断
        float ratio = light.length / light.width;
        if(ratio  < param_.light_params.min_light_ratio || 
           ratio > param_.light_params.max_light_ratio) {
            return false;
        }
        // 倾斜角判断
        cv::Point2f dir = light.top - light.bottom;
        float angle = std::abs(std::atan2(dir.x, dir.y)) * 180 / CV_PI; // atan2返回(rad) * 180/pi
        if(angle > param_.light_params.max_light_angle){
            return false;
        }
        return true;
    }

    // 配对灯条，形成装甲板
    std::vector<Armor> Detector::matchLights(const std::vector<Light>& lights){
        std::vector<Armor> armors;
        int lights_size = lights.size();
        for(int i = 0 ; i < lights_size - 1; ++i){
            for(int j = i + 1; j < lights_size; ++j){
                const Light& l1 = lights[i]; 
                const Light& l2 = lights[j];
                
                ArmorType type = isArmor(l1, l2);
                if(type == ArmorType::UNKNOWN)
                    continue;

                Armor armor;
                armor.left_light  = l1.center.x < l2.center.x ? l1 : l2;
                armor.right_light = l1.center.x < l2.center.x ? l2 : l1;
                armor.center = (l1.center + l2.center) * 0.5f;
                armor.type   = type;

                armor.points = {
                    armor.left_light.top,    // 左上
                    armor.right_light.top,   // 右上
                    armor.right_light.bottom,// 右下
                    armor.left_light.bottom  // 左下
                };
                
                armors.push_back(armor);
            }
        } 
        return armors;
    }

    // 判定装甲板类型  是否为装甲板
    ArmorType Detector::isArmor(const Light& l1, const Light& l2) const {
        // 灯条角度差
        float angle_diff = std::abs(l1.angle - l2.angle);
        if (angle_diff > param_.armor_params.max_angle_diff) {
            return ArmorType::UNKNOWN;
        }
        // 灯条长度一致性(相等)
        float length_ratio = std::min(l1.length, l2.length) / std::max(l1.length, l2.length); 
        if (length_ratio < 0.7f) {
            return ArmorType::UNKNOWN;
        }
        // 归一化中心距(用灯条自身当尺子，消除“远近”和“分辨率”的影响)
        float avg_light_length = (l1.length + l2.length) * 0.5f;
        float center_dist = cv::norm(l1.center - l2.center) / avg_light_length;

        // 判定装甲板类型
        if (center_dist >= param_.armor_params.min_small_center_dist &&
            center_dist <= param_.armor_params.max_small_center_dist) {
            return ArmorType::SMALL;
        }
        if (center_dist >= param_.armor_params.min_large_center_dist &&
            center_dist <= param_.armor_params.max_large_center_dist) {
            return ArmorType::LARGE;
        }
        return ArmorType::UNKNOWN;
    }

    // 绘制调试信息
    void Detector::drawDebug(cv::Mat& img, cv::Scalar colors) const {
        // 画灯条
        for (const auto& light : detected_lights_) {
            cv::line(img, light.top, light.bottom, cv::Scalar(255, 0, 255), 2);
        }
        // 画装甲板
        for (const auto& armor : detected_armors_) {

            const Light& left_light  = armor.left_light;
            const Light& right_light = armor.right_light;
            // 画框
            cv::line(img, left_light.top, right_light.bottom, colors, 2);
            cv::line(img, right_light.top, left_light.bottom,  colors, 2);
            cv::line(img, left_light.top, right_light.top, colors, 2);
            cv::line(img, left_light.bottom, right_light.bottom, colors, 2);

            // 画中心点
            cv::circle(img, armor.center, 4, cv::Scalar(255,0,0), -1);

            // ================= 标注装甲类型 =================
            std::string armor_text = armor.type == ArmorType::SMALL ? "SMALL" : 
                        (armor.type == ArmorType::LARGE ? "LARGE" : "UNKNOW");

            cv::Point2f text_pos = armor.center + cv::Point2f(5, -5);

            cv::putText(img, armor_text, text_pos, 
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,0,0), 2);

            // 左右灯条标记
            cv::putText(img, "L", left_light.center, cv::FONT_HERSHEY_SIMPLEX, 0.4,
                        cv::Scalar(0, 0, 255), 1);
            cv::putText(img, "R", right_light.center, cv::FONT_HERSHEY_SIMPLEX, 0.4,
                        cv::Scalar(0, 0, 255), 1);
        }
    }

    // 读取配置参数
    bool Detector::Params::loadParams(const std::string& path) {
        YAML::Node root = YAML::LoadFile(path);
        // -------- preprocess --------
        auto pre = root["preprocess"];
        prepc_params.binary_threshold = pre["binary_threshold"].as<int>();
        prepc_params.enemy_color =
            static_cast<EnemyColor>(pre["enemy_color"].as<int>());

        // -------- light --------
        auto light = root["light"];
        light_params.min_light_ratio = light["min_light_ratio"].as<float>();
        light_params.max_light_ratio = light["max_light_ratio"].as<float>();
        light_params.max_light_angle = light["max_light_angle"].as<float>();

        // -------- armor --------
        auto armor = root["armor"];
        armor_params.min_small_center_dist = armor["min_small_center_dist"].as<float>();
        armor_params.max_small_center_dist = armor["max_small_center_dist"].as<float>();
        armor_params.min_large_center_dist = armor["min_large_center_dist"].as<float>();
        armor_params.max_large_center_dist = armor["max_large_center_dist"].as<float>();
        armor_params.max_angle_diff        = armor["max_angle_diff"].as<float>();

        return true;
    }

} // namespace auto_aim