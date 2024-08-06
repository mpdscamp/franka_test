#ifndef MOTION_GENERATOR_H_
#define MOTION_GENERATOR_H_

#include <franka/robot.h>
#include <Eigen/Dense>
#include <array>
#include <cmath>

class MotionGenerator {
public:
    MotionGenerator(double speed_factor, const std::array<double, 7>& q_goal);

    franka::JointPositions operator()(const franka::RobotState& robot_state, franka::Duration period);

private:
    bool calculateDesiredValues(double t, Eigen::Matrix<double, 7, 1>* delta_q_d) const;
    void calculateSynchronizedValues();

    static constexpr double kDeltaQMotionFinished = 1e-6;
    Eigen::Matrix<double, 7, 1> q_goal_;
    Eigen::Matrix<double, 7, 1> q_start_;
    Eigen::Matrix<double, 7, 1> delta_q_;
    Eigen::Matrix<double, 7, 1> dq_max_sync_;
    Eigen::Matrix<double, 7, 1> t_1_sync_;
    Eigen::Matrix<double, 7, 1> t_2_sync_;
    Eigen::Matrix<double, 7, 1> t_f_sync_;
    Eigen::Matrix<double, 7, 1> q_1_;
    double time_ = 0.0;

    Eigen::Matrix<double, 7, 1> dq_max_;
    Eigen::Matrix<double, 7, 1> ddq_max_start_;
    Eigen::Matrix<double, 7, 1> ddq_max_goal_;
};

#endif // MOTION_GENERATOR_H_
