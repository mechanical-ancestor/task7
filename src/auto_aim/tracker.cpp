#include "task7/auto_aim/tracker.hpp"

#include <algorithm>
#include <utility>

namespace task7::auto_aim {

ArmorTracker::ArmorTracker(config::TrackerSettings settings)
    : settings_(std::move(settings)),
      filter_(6, 3, 0, CV_64F) {
    filter_.transitionMatrix = cv::Mat::eye(6, 6, CV_64F);
    filter_.measurementMatrix = cv::Mat::zeros(3, 6, CV_64F);
    filter_.measurementMatrix.at<double>(0, 0) = 1.0;
    filter_.measurementMatrix.at<double>(1, 1) = 1.0;
    filter_.measurementMatrix.at<double>(2, 2) = 1.0;

    filter_.processNoiseCov = cv::Mat::eye(6, 6, CV_64F);
    filter_.processNoiseCov.at<double>(0, 0) = settings_.process_noise_position;
    filter_.processNoiseCov.at<double>(1, 1) = settings_.process_noise_position;
    filter_.processNoiseCov.at<double>(2, 2) = settings_.process_noise_position;
    filter_.processNoiseCov.at<double>(3, 3) = settings_.process_noise_velocity;
    filter_.processNoiseCov.at<double>(4, 4) = settings_.process_noise_velocity;
    filter_.processNoiseCov.at<double>(5, 5) = settings_.process_noise_velocity;

    filter_.measurementNoiseCov = cv::Mat::eye(3, 3, CV_64F) * settings_.measurement_noise;
    filter_.errorCovPost = cv::Mat::eye(6, 6, CV_64F) * 1e-1;
    filter_.statePost = cv::Mat::zeros(6, 1, CV_64F);
    filter_.statePre = filter_.statePost.clone();
}

void ArmorTracker::reset(const cv::Point3f& position, TimePoint now) {
    filter_.statePost = cv::Mat::zeros(6, 1, CV_64F);
    filter_.statePost.at<double>(0, 0) = position.x;
    filter_.statePost.at<double>(1, 0) = position.y;
    filter_.statePost.at<double>(2, 0) = position.z;
    filter_.statePre = filter_.statePost.clone();
    filter_.errorCovPost = cv::Mat::eye(6, 6, CV_64F) * 1e-1;
    last_update_time_ = now;
    initialized_ = true;
}

bool ArmorTracker::update(const cv::Point3f& position, TimePoint now) {
    if (!initialized_) {
        reset(position, now);
        return false;
    }

    const double dt = std::max(std::chrono::duration<double>(now - last_update_time_).count(), 1e-3);
    updateTransition(dt);
    filter_.predict();

    cv::Mat measurement = (cv::Mat_<double>(3, 1) << position.x, position.y, position.z);
    filter_.correct(measurement);
    last_update_time_ = now;
    return true;
}

cv::Point3f ArmorTracker::predictFuture(double lead_seconds) const {
    if (!initialized_) {
        return {};
    }

    const cv::Mat& state = filter_.statePost;
    return cv::Point3f(
        static_cast<float>(state.at<double>(0, 0) + state.at<double>(3, 0) * lead_seconds),
        static_cast<float>(state.at<double>(1, 0) + state.at<double>(4, 0) * lead_seconds),
        static_cast<float>(state.at<double>(2, 0) + state.at<double>(5, 0) * lead_seconds));
}

void ArmorTracker::markLostIfExpired(TimePoint now) {
    if (!initialized_) {
        return;
    }

    if (std::chrono::duration<double>(now - last_update_time_).count() > settings_.lost_reset_seconds) {
        clear();
    }
}

void ArmorTracker::clear() {
    filter_.statePost = cv::Mat::zeros(6, 1, CV_64F);
    filter_.statePre = filter_.statePost.clone();
    filter_.errorCovPost = cv::Mat::eye(6, 6, CV_64F) * 1e-1;
    initialized_ = false;
}

bool ArmorTracker::hasTrack() const {
    return initialized_;
}

void ArmorTracker::updateTransition(double dt) {
    filter_.transitionMatrix = cv::Mat::eye(6, 6, CV_64F);
    filter_.transitionMatrix.at<double>(0, 3) = dt;
    filter_.transitionMatrix.at<double>(1, 4) = dt;
    filter_.transitionMatrix.at<double>(2, 5) = dt;
}

}  // namespace task7::auto_aim
