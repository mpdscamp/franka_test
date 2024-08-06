#include "motion_generator.h"

MotionGenerator::MotionGenerator(double speed_factor, const std::array<double, 7>& q_goal)
    : q_goal_(q_goal.data()) {
    dq_max_ = (Eigen::Matrix<double, 7, 1>() << 2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5).finished() * speed_factor;
    ddq_max_start_ = (Eigen::Matrix<double, 7, 1>() << 5, 5, 5, 5, 5, 5, 5).finished() * speed_factor;
    ddq_max_goal_ = (Eigen::Matrix<double, 7, 1>() << 5, 5, 5, 5, 5, 5, 5).finished() * speed_factor;
    dq_max_sync_.setZero();
    q_start_.setZero();
    delta_q_.setZero();
    t_1_sync_.setZero();
    t_2_sync_.setZero();
    t_f_sync_.setZero();
    q_1_.setZero();
}

bool MotionGenerator::calculateDesiredValues(double t, Eigen::Matrix<double, 7, 1>* delta_q) const {
    for (size_t i = 0; i < 7; i++) {
        if (t < t_1_sync_[i]) {
            (*delta_q)[i] = q_start_[i] + 0.5 * ddq_max_start_[i] * std::pow(t, 2);
        } else if (t < t_2_sync_[i]) {
            (*delta_q)[i] = q_1_[i] + dq_max_sync_[i] * (t - t_1_sync_[i]);
        } else if (t <= t_f_sync_[i]) {
            double t_decel = t - t_2_sync_[i];
            (*delta_q)[i] = q_1_[i] + dq_max_sync_[i] * (t_2_sync_[i] - t_1_sync_[i]) + 
                            dq_max_sync_[i] * t_decel - 0.5 * ddq_max_goal_[i] * std::pow(t_decel, 2);
        } else {
            (*delta_q)[i] = q_goal_[i];
        }
    }
    return t >= t_f_sync_.maxCoeff();
}

void MotionGenerator::calculateSynchronizedValues() {
    Eigen::Matrix<double, 7, 1> t_f_new;
    for (size_t i = 0; i < 7; i++) {
        t_f_new[i] = std::abs(delta_q_[i] / dq_max_[i]);
    }

    double t_f_max = t_f_new.maxCoeff();
    for (size_t i = 0; i < 7; i++) {
        dq_max_sync_[i] = std::abs(delta_q_[i]) / t_f_max;
        t_1_sync_[i] = dq_max_sync_[i] / ddq_max_start_[i];
        t_2_sync_[i] = t_f_max - dq_max_sync_[i] / ddq_max_goal_[i];
        t_f_sync_[i] = t_f_max;

        q_1_[i] = q_start_[i] + 0.5 * ddq_max_start_[i] * std::pow(t_1_sync_[i], 2);
    }
}

franka::JointPositions MotionGenerator::operator()(const franka::RobotState& robot_state, franka::Duration period) {
    time_ += period.toSec();
    Eigen::Matrix<double, 7, 1> delta_q;
    bool motion_finished = calculateDesiredValues(time_, &delta_q);

    std::array<double, 7> q_desired;
    Eigen::VectorXd::Map(&q_desired[0], 7) = q_start_ + delta_q;
    return motion_finished ? franka::MotionFinished(franka::JointPositions(q_desired)) : franka::JointPositions(q_desired);
}
