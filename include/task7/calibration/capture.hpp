#pragma once

#include "task7/config/config.hpp"

namespace task7::calibration {

int runChessboardCapture(
    const config::CameraSettings& camera_settings,
    const config::CalibrationSettings& calibration_settings);

}  // namespace task7::calibration
