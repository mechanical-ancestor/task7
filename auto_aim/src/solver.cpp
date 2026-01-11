#include <yaml-cpp/yaml.h>
#include <iostream>
#include <stdexcept>

#include "solver.hpp"
#include "type.hpp"

namespace auto_aim {

Solver::Solver(const std::string& path)
{
    try {
        YAML::Node yaml = YAML::LoadFile(path);
        auto c = yaml["camera_matrix"].as<std::vector<double>>();
        auto d = yaml["dist_coeffs"].as<std::vector<double>>();

        if (c.size() != 9 || d.size() != 5) {
            throw std::runtime_error("calib param size error");
        }

        camera_matrix_ = cv::Mat(3, 3, CV_64F, c.data()).clone();
        dist_coeffs_   = cv::Mat(1, 5, CV_64F, d.data()).clone();

    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Solver init failed: " << e.what() << std::endl;
        // 初始化失败，后续调用自动跳过
        return;
    }
    // Unit: m
    constexpr double small_half_y = SMALL_ARMOR_WIDTH  / 2.0 ;
    constexpr double small_half_z = SMALL_ARMOR_HEIGHT / 2.0 ;
    constexpr double large_half_y = LARGE_ARMOR_WIDTH  / 2.0 ;
    constexpr double large_half_z = LARGE_ARMOR_HEIGHT / 2.0 ;


    // coordina: x right, y down, z forward
    small_armor_pts_ = {
        {0, -small_half_y ,  small_half_z },
        {0,  small_half_y ,  small_half_z },
        {0,  small_half_y , -small_half_z },
        {0, -small_half_y , -small_half_z }
    };

    big_armor_pts_ = {
        {0, -large_half_y ,  large_half_z },
        {0,  large_half_y ,  large_half_z },
        {0,  large_half_y , -large_half_z },
        {0, -large_half_y , -large_half_z }
    };
    // small_armor_pts_ = {
    //     {0,  SMALL_W / 2,  LIGHTBAR / 2},
    //     {0, -SMALL_W / 2,  LIGHTBAR / 2},
    //     {0, -SMALL_W / 2, -LIGHTBAR / 2},
    //     {0,  SMALL_W / 2, -LIGHTBAR / 2}
    // };

    // big_armor_pts_ = {
    //     {0,  BIG_W / 2,  LIGHTBAR / 2},
    //     {0, -BIG_W / 2,  LIGHTBAR / 2},
    //     {0, -BIG_W / 2, -LIGHTBAR / 2},
    //     {0,  BIG_W / 2, -LIGHTBAR / 2}
    // };

    // std::cout << "[INFO] Solver initialized successfully!" << std::endl;
}
// ============= 解算 ================ //
bool Solver::solve(Armor& armor) const { 
    if (camera_matrix_.empty() || dist_coeffs_.empty()) {
        std::cerr << "[WARN] Solver not initialized" << std::endl;
        return false;
    }
    std::vector<cv::Point2f> image_armor_points;
    
    // Fill in image points
    // image_armor_points.emplace_back(armor.left_light.bottom);
    image_armor_points.emplace_back(armor.left_light.top);
    image_armor_points.emplace_back(armor.right_light.top);
    image_armor_points.emplace_back(armor.right_light.bottom);
    image_armor_points.emplace_back(armor.left_light.bottom);

    // if (armor.points.empty() || armor.points.size() != 4) {
    //     std::cout << armor.points.size() << std::endl; 
    //     return false; 
    // }

    auto& object_pts = (armor.type == ArmorType::LARGE) ? big_armor_pts_ : small_armor_pts_;
    cv::Vec3d rvec, tvec; // Rotation_vector , Translation_vector  armor 相对于 camera  单位 m 
    bool ok = cv::solvePnP(
        object_pts,
        image_armor_points,
        camera_matrix_,
        dist_coeffs_,
        rvec,
        tvec,
        false,
        cv::SOLVEPNP_IPPE
    );

    if (ok) { // 储存结果
        armor.position = cv::Point3f(static_cast<float>(tvec[0]),
                                     static_cast<float>(tvec[1]),
                                     static_cast<float>(tvec[2]) );
        armor.points = image_armor_points;
        armor.rvec = rvec;
        armor.tvec = tvec;
    }

    return ok;
}

// 将 3D 点投影到图像平面
void Solver::projectPoints(const std::vector<cv::Point3f>& obj_pts,
                           const cv::Vec3d& rvec,
                           const cv::Vec3d& tvec,
                           std::vector<cv::Point2f>& img_pts) const
{
    img_pts.clear();
    if (camera_matrix_.empty() || dist_coeffs_.empty()) return;
    cv::projectPoints(obj_pts, rvec, tvec, camera_matrix_, dist_coeffs_, img_pts);
}

// 根据 armor 的类型选择物体坐标并投影得到四个像素点
void Solver::projectArmorPoints(const Armor& armor,
                                const cv::Vec3d& rvec,
                                const cv::Vec3d& tvec,
                                std::vector<cv::Point2f>& img_pts) const
{
    if (armor.type == ArmorType::LARGE) {
        projectPoints(big_armor_pts_, rvec, tvec, img_pts);
    } else {
        projectPoints(small_armor_pts_, rvec, tvec, img_pts);
    }
}


// void Solver::printSolverDebug(cv::Mat& frame, const std::vector<Armor>& armors) {
//     if (frame.empty()) return;

//     for (const auto& armor : armors) {
//         if (armor.points.empty() || armor.points.size() != 4) {
//             continue;
//         }

//         // for (int i = 0; i < 4; ++i) {
//         //     cv::line(frame, armor.points[i], armor.points[(i+1)%4], cv::Scalar(0, 255, 0), 2);
//         // }

//         // cv::circle(frame, armor.center, 4, cv::Scalar(0, 0, 255), -1);
//         // std::cout << "Test 1 pass " << std::endl; 

//         std::cout << "( " 
//         << armor.position.x << ", " 
//         << armor.position.y << ", " 
//         << armor.position.z << " )" 
//         << std::endl;        
//     }
// }

float Solver::calculateDistanceToCenter(const cv::Point2f & image_point)
{
    float cx = camera_matrix_.at<double>(0, 2);
    float cy = camera_matrix_.at<double>(1, 2);
    return cv::norm(image_point - cv::Point2f(cx, cy));
}
} // namespace auto_aim