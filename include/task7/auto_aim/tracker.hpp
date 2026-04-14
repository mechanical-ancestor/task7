#pragma once

#include <opencv2/core.hpp>
#include <opencv2/video/tracking.hpp>

#include <chrono>

#include "task7/config/config.hpp"

namespace task7::auto_aim {

class ArmorTracker {
   public:
    using Clock = std::chrono::steady_clock;
    using TimePoint = std::chrono::time_point<Clock>;

    explicit ArmorTracker(config::TrackerSettings settings);

    void reset(const cv::Point3f& position, TimePoint now);
    bool update(const cv::Point3f& position, TimePoint now);
    cv::Point3f predictFuture(double lead_seconds) const;
    void markLostIfExpired(TimePoint now);
    void clear();

    bool hasTrack() const;

   private:
    void updateTransition(double dt);

    config::TrackerSettings settings_;
    cv::KalmanFilter filter_;
    TimePoint last_update_time_;
    bool initialized_{false};
};

}  // namespace task7::auto_aim
