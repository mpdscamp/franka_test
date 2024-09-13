#ifndef CARTESIAN_MOTION_GENERATOR_H_
#define CARTESIAN_MOTION_GENERATOR_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <franka/robot_state.h>

class CartesianMotionGenerator {
 public:
  CartesianMotionGenerator(double speed_factor, const Eigen::Vector3d& position_goal, const Eigen::Quaterniond& orientation_goal);

  bool generatePose(double time, Eigen::Vector3d* position, Eigen::Quaterniond* orientation) const;

 private:
  void calculateSynchronizedValues();

  static constexpr double kDeltaMotionFinished = 1e-6;
  Eigen::Vector3d position_goal_;
  Eigen::Quaterniond orientation_goal_;

  Eigen::Vector3d position_start_;
  Eigen::Quaterniond orientation_start_;
  Eigen::Vector3d delta_position_;

  Eigen::Vector3d max_velocity_sync_;
  Eigen::Vector3d t_1_sync_;
  Eigen::Vector3d t_2_sync_;
  Eigen::Vector3d t_f_sync_;
  Eigen::Vector3d position_1_;

  double time_ = 0.0;
  double speed_factor_;

  Eigen::Vector3d max_velocity_ = Eigen::Vector3d(0.5, 0.5, 0.5);
  Eigen::Vector3d max_acceleration_ = Eigen::Vector3d(1.0, 1.0, 1.0);
};

#endif // CARTESIAN_MOTION_GENERATOR_H_
