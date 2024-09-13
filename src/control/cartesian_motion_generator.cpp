#include "control/cartesian_motion_generator.h"
#include <cmath>

CartesianMotionGenerator::CartesianMotionGenerator(double speed_factor, const Eigen::Vector3d& position_goal, const Eigen::Quaterniond& orientation_goal)
    : position_goal_(position_goal), orientation_goal_(orientation_goal), speed_factor_(speed_factor) {
  max_velocity_ *= speed_factor;
  max_acceleration_ *= speed_factor;
  max_velocity_sync_.setZero();
  position_start_.setZero();
  delta_position_.setZero();
  t_1_sync_.setZero();
  t_2_sync_.setZero();
  t_f_sync_.setZero();
  position_1_.setZero();
}

void CartesianMotionGenerator::calculateSynchronizedValues() {
  Eigen::Vector3d max_velocity_reach(max_velocity_);
  Eigen::Vector3d t_f = Eigen::Vector3d::Zero();
  Eigen::Vector3d delta_t_2 = Eigen::Vector3d::Zero();
  Eigen::Vector3d t_1 = Eigen::Vector3d::Zero();
  Eigen::Vector3d delta_t_2_sync = Eigen::Vector3d::Zero();

  for (size_t i = 0; i < 3; i++) {
    if (std::abs(delta_position_[i]) > kDeltaMotionFinished) {
      if (std::abs(delta_position_[i]) < (3.0 / 4.0 * (std::pow(max_velocity_[i], 2.0) / max_acceleration_[i]))) {
        max_velocity_reach[i] = std::sqrt(4.0 / 3.0 * delta_position_[i] * (max_acceleration_[i]));
      }
      t_1[i] = 1.5 * max_velocity_reach[i] / max_acceleration_[i];
      delta_t_2[i] = 1.5 * max_velocity_reach[i] / max_acceleration_[i];
      t_f[i] = t_1[i] / 2.0 + delta_t_2[i] / 2.0 + std::abs(delta_position_[i]) / max_velocity_reach[i];
    }
  }
  double max_t_f = t_f.maxCoeff();
  for (size_t i = 0; i < 3; i++) {
    if (std::abs(delta_position_[i]) > kDeltaMotionFinished) {
      double a = 1.5 / 2.0 * max_acceleration_[i];
      double b = -1.0 * max_t_f * max_acceleration_[i];
      double c = std::abs(delta_position_[i]) * max_acceleration_[i];
      double delta = b * b - 4.0 * a * c;
      if (delta < 0.0) {
        delta = 0.0;
      }
      max_velocity_sync_[i] = (-1.0 * b - std::sqrt(delta)) / (2.0 * a);
      t_1_sync_[i] = 1.5 * max_velocity_sync_[i] / max_acceleration_[i];
      delta_t_2_sync[i] = 1.5 * max_velocity_sync_[i] / max_acceleration_[i];
      t_f_sync_[i] = t_1_sync_[i] / 2.0 + delta_t_2_sync[i] / 2.0 + std::abs(delta_position_[i] / max_velocity_sync_[i]);
      t_2_sync_[i] = t_f_sync_[i] - delta_t_2_sync[i];
      position_1_[i] = max_velocity_sync_[i] * (0.5 * t_1_sync_[i]);
    }
  }
}

bool CartesianMotionGenerator::generatePose(double time, Eigen::Vector3d* position, Eigen::Quaterniond* orientation) const {
  Eigen::Vector3d delta_position_d;
  Eigen::Vector3d sign_delta_position = delta_position_.cwiseSign();

  Eigen::Vector3d t_d = t_2_sync_ - t_1_sync_;
  Eigen::Vector3d delta_t_2_sync = t_f_sync_ - t_2_sync_;
  std::array<bool, 3> motion_finished{};

  for (size_t i = 0; i < 3; i++) {
    if (std::abs(delta_position_[i]) < kDeltaMotionFinished) {
      delta_position_d[i] = 0;
      motion_finished[i] = true;
    } else {
      if (time < t_1_sync_[i]) {
        delta_position_d[i] = -1.0 / std::pow(t_1_sync_[i], 3.0) * max_velocity_sync_[i] * sign_delta_position[i] *
                              (0.5 * time - t_1_sync_[i]) * std::pow(time, 3.0);
      } else if (time >= t_1_sync_[i] && time < t_2_sync_[i]) {
        delta_position_d[i] = position_1_[i] + (time - t_1_sync_[i]) * max_velocity_sync_[i] * sign_delta_position[i];
      } else if (time >= t_2_sync_[i] && time < t_f_sync_[i]) {
        delta_position_d[i] =
            delta_position_[i] + 0.5 *
                                 (1.0 / std::pow(delta_t_2_sync[i], 3.0) *
                                  (time - t_1_sync_[i] - 2.0 * delta_t_2_sync[i] - t_d[i]) *
                                  std::pow((time - t_1_sync_[i] - t_d[i]), 3.0) +
                                  (2.0 * time - 2.0 * t_1_sync_[i] - delta_t_2_sync[i] - 2.0 * t_d[i])) *
                                 max_velocity_sync_[i] * sign_delta_position[i];
      } else {
        delta_position_d[i] = delta_position_[i];
        motion_finished[i] = true;
      }
    }
  }

  *position = position_start_ + delta_position_d;

  // Orientation interpolation
  if (time >= t_f_sync_.maxCoeff()) {
    *orientation = orientation_goal_;
  } else {
    double t_normalized = time / t_f_sync_.maxCoeff();
    *orientation = orientation_start_.slerp(t_normalized, orientation_goal_);
  }

  return std::all_of(motion_finished.cbegin(), motion_finished.cend(), [](bool x) { return x; });
}
