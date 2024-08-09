#ifndef MOTION_GENERATOR_H_
#define MOTION_GENERATOR_H_

#include <franka/robot.h>
#include <Eigen/Dense>
#include <array>
#include <cmath>

using Vector7d = Eigen::Matrix<double, 7, 1>;
using Vector7i = Eigen::Matrix<int, 7, 1>;

class MotionGenerator {
public:
    MotionGenerator(double speed_factor, const std::array<double, 7>& q_goal, 
                                 const std::array<double, 7>& dq_max, 
                                 const std::array<double, 7>& ddq_max_start, 
                                 const std::array<double, 7>& ddq_max_goal);

    franka::JointPositions operator()(const franka::RobotState& robot_state, franka::Duration period);

private:
    bool calculateDesiredValues(double t, Vector7d* delta_q_d) const;
    void calculateSynchronizedValues();

    static constexpr double kDeltaQMotionFinished = 1e-6;
    Vector7d q_goal_;
    Vector7d q_start_;
    Vector7d delta_q_;
    Vector7d dq_max_sync_;
    Vector7d t_1_sync_;
    Vector7d t_2_sync_;
    Vector7d t_f_sync_;
    Vector7d q_1_;
    double time_ = 0.0;

    Vector7d dq_max_;
    Vector7d ddq_max_start_;
    Vector7d ddq_max_goal_;
};

#endif // MOTION_GENERATOR_H_
