#include <yaml-cpp/yaml.h>
#include <iostream>
#include <stdexcept>

#include "solver.hpp"
#include "type.hpp"

namespace auto_aim {

Solver::Solver(const std::string& calib_path)
{
    try {
        YAML::Node yaml = YAML::LoadFile(calib_path);
        auto c = yaml["camera_matrix"]["data"].as<std::vector<double>>();
        auto d = yaml["dist_coeffs"]["data"].as<std::vector<double>>();

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

    constexpr float LIGHTBAR = 56e-3f;
    constexpr float SMALL_W  = 135e-3f;
    constexpr float BIG_W    = 230e-3f;

    small_armor_pts_ = {
        {0,  SMALL_W / 2,  LIGHTBAR / 2},
        {0, -SMALL_W / 2,  LIGHTBAR / 2},
        {0, -SMALL_W / 2, -LIGHTBAR / 2},
        {0,  SMALL_W / 2, -LIGHTBAR / 2}
    };

    big_armor_pts_ = {
        {0,  BIG_W / 2,  LIGHTBAR / 2},
        {0, -BIG_W / 2,  LIGHTBAR / 2},
        {0, -BIG_W / 2, -LIGHTBAR / 2},
        {0,  BIG_W / 2, -LIGHTBAR / 2}
    };

}

bool Solver::solve(Armor& armor) const { 
    if (camera_matrix_.empty() || dist_coeffs_.empty()) {
        std::cerr << "[WARN] Solver not initialized" << std::endl;
        return false;
    }
    if (armor.points.empty() || armor.points.size() != 4) {
        std::cout << armor.points.size() << std::endl; 
        return false; 
    }

    const auto& object_pts = (armor.type == ArmorType::LARGE) ? big_armor_pts_ : small_armor_pts_;
    cv::Vec3d rvec, tvec; 

    bool ok = cv::solvePnP(
        object_pts,
        armor.points,
        camera_matrix_,
        dist_coeffs_,
        rvec,
        tvec,
        false,
        cv::SOLVEPNP_IPPE
    );

    if (!ok) return false;

    armor.position = cv::Point3f(
        static_cast<float>(tvec[0]), // 相机坐标轴下的 x 
        static_cast<float>(tvec[1]), // 相机坐标轴下的 y
        static_cast<float>(tvec[2])  // 相机坐标轴下的 z
    );
    return true;
}

void Solver::printSolverDebug(cv::Mat& frame, const std::vector<Armor>& armors) {
    if (frame.empty()) return;

    for (const auto& armor : armors) {
        if (armor.points.empty() || armor.points.size() != 4) {
            continue;
        }

        std::cout << "( " 
        << armor.position.x << ", " 
        << armor.position.y << ", " 
        << armor.position.z << " )" 
        << std::endl;        
    }
}

} // namespace auto_aim