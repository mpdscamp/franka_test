#ifndef FRANKA_CONTROLLER_H_
#define FRANKA_CONTROLLER_H_

#include <franka/robot.h>
#include <franka/model.h>
#include <franka/exception.h>
#include <nlohmann/json.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>
#include <atomic>
#include <array>
#include <memory>
#include <cmath>
#include <thread>
#include "utils/logger.h"
#include "utils/state_serializer.h"
#include "communication/central_communication_hub.h"
#include "control/cartesian_motion_generator.h"

class FrankaController {
public:
    FrankaController(const nlohmann::json& parameters, const std::string& mq_name, int robot_id);
    ~FrankaController();
    void run();

private:
    void loadParameters(const nlohmann::json& parameters);
    void sendState(const franka::RobotState& state);
    void performAutomaticErrorRecovery(franka::Robot& robot);
    void logRobotState(const franka::RobotState& state, Eigen::VectorXd tau_d);

    // Advanced control methods
    void applyCartesianImpedanceControl(franka::Robot& robot, const franka::Model& model);
    void setFiltering(double update_frequency, double filter_params_nullspace_config, double filter_params_stiffness,
                      double filter_params_pose, double filter_params_wrench);
    void updateFilteredPose();
    void updateFilteredStiffness();
    void updateFilteredNullspaceConfig();
    void updateFilteredWrench();
    Eigen::VectorXd calculateCommandedTorques(const franka::RobotState& state, const franka::Model& model);
    void saturateTorqueRate(const Eigen::VectorXd& tau_d_calculated, Eigen::VectorXd* tau_d_saturated, double delta_tau_max);
    void pseudoInverse(const Eigen::MatrixXd& matrix, Eigen::MatrixXd* pseudo_inverse);

    // Utility functions
    void setStiffness(double translational_stiffness, double rotational_stiffness, bool auto_damping = true);
    void setStiffness(double t_x, double t_y, double t_z, double r_x, double r_y, double r_z, bool auto_damping);
    void setStiffness(const Eigen::Matrix<double, 7, 1>& stiffness, bool auto_damping);
    void setDampingFactors(double d_x, double d_y, double d_z, double d_a, double d_b, double d_c, double d_n);
    void applyDamping();
    double dampingRule(double stiffness) const;
    Eigen::Vector3d calculateOrientationError(const Eigen::Quaterniond& orientation_d, Eigen::Quaterniond orientation);

    // Control inputs and outputs
    std::string robot_hostname_;
    int robot_id_;
    Eigen::Vector3d position_d_;
    Eigen::Quaterniond orientation_d_;
    Eigen::Vector3d position_d_target_;
    Eigen::Quaterniond orientation_d_target_;

    std::array<double, 7> real_start_pose_;
    std::array<double, 7> start_pose_;
    std::array<double, 7> end_pose_;
    Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
    Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
    Eigen::Matrix<double, 6, 6> cartesian_damping_;
    Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
    Eigen::VectorXd q_d_nullspace_;
    Eigen::VectorXd q_d_nullspace_target_;
    Eigen::VectorXd tau_c_;  // Commanded torques
    Eigen::MatrixXd jacobian_;  // Robot Jacobian

    Eigen::Matrix<double, 6, 1> cartesian_wrench_;  // Current Cartesian wrench
    Eigen::Matrix<double, 6, 1> cartesian_wrench_target_;  // Target Cartesian wrench

    Eigen::Matrix<double, 7, 1> damping_factors_; // Damping factors

    double speed_factor_;
    double translational_stiffness_;
    double rotational_stiffness_;
    double delta_tau_max_;  // Max allowed change in torque
    double nullspace_stiffness_;
    double nullspace_damping_;

    // Filtering parameters
    double update_frequency_;  // Frequency for the filtering operations
    double filter_params_nullspace_config_;  // Filtering parameter for nullspace configuration
    double filter_params_stiffness_;  // Filtering parameter for stiffness
    double filter_params_pose_;  // Filtering parameter for pose
    double filter_params_wrench_;  // Filtering parameter for wrench

    std::atomic_bool running_;
    std::unique_ptr<CentralCommunicationHub> comm_hub_;
    std::chrono::steady_clock::time_point start_time_;
    std::chrono::steady_clock::time_point send_time_;
    FILE* output_file_;

    std::unique_ptr<CartesianMotionGenerator> motion_generator_;

    enum class MotionPhase { MOVE_TO_START, MOVE_TO_END };
    MotionPhase current_phase_;

    double time_;  // Added for keeping track of time in the control loop
};

#endif // FRANKA_CONTROLLER_H_
