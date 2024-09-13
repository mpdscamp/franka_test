#include "control/franka_controller.h"

FrankaController::FrankaController(const nlohmann::json& parameters, const std::string& mq_name, int robot_id)
    : robot_id_(robot_id),
      running_(true),
      current_phase_(MotionPhase::MOVE_TO_START),
      output_file_(nullptr) {

    // Initialize matrices and variables with zeros to avoid uninitialized memory issues
    q_d_nullspace_.setZero(7);
    q_d_nullspace_target_.setZero(7);
    tau_c_.setZero(7);
    jacobian_.setZero(6, 7);
    cartesian_wrench_target_.setZero(6);
    cartesian_wrench_.setZero(6);
    cartesian_stiffness_.setZero();
    cartesian_stiffness_target_.setZero();
    cartesian_damping_.setZero();
    cartesian_damping_target_.setZero();
    nullspace_stiffness_ = 0.0;
    nullspace_damping_ = 0.0;

    // Load parameters from the JSON file
    loadParameters(parameters);

    comm_hub_ = std::make_unique<CentralCommunicationHub>(mq_name);

    // Open the output file for logging
    output_file_ = fopen(("../robot_" + std::to_string(robot_id) + "_log.csv").c_str(), "w");
    if (output_file_) {
        fprintf(output_file_, "time,q[0],q[1],q[2],q[3],q[4],q[5],q[6],"
                              "O_T_EE[0],O_T_EE[1],O_T_EE[2],O_T_EE[3],O_T_EE[4],O_T_EE[5],O_T_EE[6],O_T_EE[7],"
                              "O_T_EE[8],O_T_EE[9],O_T_EE[10],O_T_EE[11],O_T_EE[12],O_T_EE[13],O_T_EE[14],O_T_EE[15],"
                              "x,y,z,dx,dy,dz,"
                              "tau[0],tau[1],tau[2],tau[3],tau[4],tau[5],tau[6]\n");
    }
}

FrankaController::~FrankaController() {
    running_ = false;  // Ensure the control loop exits

    // Ensure the robot is stopped
    try {
        franka::Robot robot(robot_hostname_);
        robot.stop();  // This stops the robot's motion
    } catch (const franka::Exception& ex) {
        Logger::getInstance().log(Logger::Level::ERROR, "Error during robot stop: " + std::string(ex.what()));
    }

    if (output_file_) {
        fclose(output_file_);
    }
}

void FrankaController::loadParameters(const nlohmann::json& parameters) {
    robot_hostname_ = parameters.value("robot_hostname", robot_hostname_);

    const auto& start_position = parameters["start_pose"]["position"];
    const auto& start_orientation = parameters["start_pose"]["orientation"];
    if (start_position.size() != 3 || start_orientation.size() != 4) {
        throw std::runtime_error("start_pose position should have 3 elements and orientation should have 4 elements.");
    }
    start_pose_ = {
        start_position[0], start_position[1], start_position[2],
        start_orientation[0], start_orientation[1], start_orientation[2], start_orientation[3]
    };

    const auto& end_position = parameters["end_pose"]["position"];
    const auto& end_orientation = parameters["end_pose"]["orientation"];
    if (end_position.size() != 3 || end_orientation.size() != 4) {
        throw std::runtime_error("end_pose position should have 3 elements and orientation should have 4 elements.");
    }
    end_pose_ = {
        end_position[0], end_position[1], end_position[2],
        end_orientation[0], end_orientation[1], end_orientation[2], end_orientation[3]
    };

    speed_factor_ = parameters.value("speed_factor", 0.5);
    translational_stiffness_ = parameters.value("translational_stiffness", 1000.0);
    rotational_stiffness_ = parameters.value("rotational_stiffness", 100.0);
    delta_tau_max_ = parameters.value("delta_tau_max", 1.0);

    update_frequency_ = parameters.value("update_frequency", 1000.0);
    filter_params_nullspace_config_ = parameters.value("filter_params_nullspace_config", 0.1);
    filter_params_stiffness_ = parameters.value("filter_params_stiffness", 0.1);
    filter_params_pose_ = parameters.value("filter_params_pose", 0.1);
    filter_params_wrench_ = parameters.value("filter_params_wrench", 0.1);

    const auto& damping_factors = parameters["damping_factors"];
    if (damping_factors.size() != 7) {
        throw std::runtime_error("damping_factors should have 7 elements.");
    }

    setDampingFactors(damping_factors[0], damping_factors[1], damping_factors[2], 
                      damping_factors[3], damping_factors[4], damping_factors[5], damping_factors[6]);

    // Apply loaded or default stiffness values
    setStiffness(translational_stiffness_, rotational_stiffness_, true);

    // Initialize the Cartesian motion generator
    Eigen::Vector3d end_pos(end_pose_[0], end_pose_[1], end_pose_[2]);
    Eigen::Quaterniond end_ori(end_pose_[3], end_pose_[4], end_pose_[5], end_pose_[6]);
    motion_generator_ = std::make_unique<CartesianMotionGenerator>(0.5, end_pos, end_ori);
}

void FrankaController::setStiffness(double t_x, double t_y, double t_z, double r_x, double r_y, double r_z, bool auto_damping) {
    Eigen::Matrix<double, 7, 1> stiffness_vector;
    stiffness_vector << t_x, t_y, t_z, r_x, r_y, r_z, 0.0;  // Nullspace stiffness is set to 0 by default
    setStiffness(stiffness_vector, auto_damping);
}

void FrankaController::setStiffness(double translational_stiffness, double rotational_stiffness, bool auto_damping) {
    Eigen::Matrix<double, 7, 1> stiffness_vector;
    stiffness_vector << translational_stiffness, translational_stiffness, translational_stiffness, 
                         rotational_stiffness, rotational_stiffness, rotational_stiffness, 0.0;
    setStiffness(stiffness_vector, auto_damping);
}

void FrankaController::setStiffness(const Eigen::Matrix<double, 7, 1>& stiffness, bool auto_damping) {    
    for (int i = 0; i < 6; ++i) {
        cartesian_stiffness_target_(i, i) = std::max(stiffness(i), 0.0);
    }
    nullspace_stiffness_ = std::max(stiffness(6), 0.0);

    if (auto_damping) {
        applyDamping();
    }
}

void FrankaController::setDampingFactors(double d_x, double d_y, double d_z, double d_a, double d_b, double d_c, double d_n) {
    Eigen::Matrix<double, 7, 1> damping_factors;
    damping_factors << d_x, d_y, d_z, d_a, d_b, d_c, d_n;

    for (int i = 0; i < 7; ++i) {
        damping_factors_(i) = std::max(damping_factors(i), 0.0);
    }

    applyDamping();
}

void FrankaController::applyDamping() {
    for (int i = 0; i < 6; i++) {
        cartesian_damping_target_(i, i) = damping_factors_(i) * dampingRule(cartesian_stiffness_target_(i, i));
    }
    nullspace_damping_ = damping_factors_(6) * dampingRule(nullspace_stiffness_);
}

double FrankaController::dampingRule(double stiffness) const {
    return 2 * std::sqrt(stiffness);
}

Eigen::Vector3d FrankaController::calculateOrientationError(const Eigen::Quaterniond& orientation_d, Eigen::Quaterniond orientation) {
    if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
        orientation.coeffs() << -orientation.coeffs();
    }
    const Eigen::Quaterniond error_quaternion(orientation * orientation_d.inverse());
    Eigen::AngleAxisd error_quaternion_angle_axis(error_quaternion);
    return error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();
}

void FrankaController::logRobotState(const franka::RobotState& state, Eigen::VectorXd tau_d) {
    if (output_file_) {
        auto current_time = std::chrono::steady_clock::now();
        double elapsed_time = std::chrono::duration<double>(current_time - start_time_).count();
        
        fprintf(output_file_, "%.6f,", elapsed_time);
        for (int i = 0; i < 7; ++i) {
            fprintf(output_file_, "%.6f,", state.q[i]);
        }
        for (int i = 0; i < 16; ++i) {
            fprintf(output_file_, "%.6f,", state.O_T_EE[i]);
        }
        
        // Log the x, y, z positions
        fprintf(output_file_, "%.6f,%.6f,%.6f,", state.O_T_EE[12], state.O_T_EE[13], state.O_T_EE[14]);
        
        // Log the dx, dy, dz velocities
        fprintf(output_file_, "%.6f,%.6f,%.6f,", state.O_T_EE_d[0], state.O_T_EE_d[1], state.O_T_EE_d[2]);

        // Log the torques
        for (int i = 0; i < 7; ++i) {
            fprintf(output_file_, "%.6f,", tau_d[i]);
        }
        fprintf(output_file_, "\n");
    }
}

void FrankaController::sendState(const franka::RobotState& state) {
    std::string state_message = StateSerializer::serialize(state);
    send_time_ = std::chrono::steady_clock::now();
    comm_hub_->sendState(state_message, robot_id_);
}

void FrankaController::performAutomaticErrorRecovery(franka::Robot& robot) {
    try {
        robot.automaticErrorRecovery();
        Logger::getInstance().log(Logger::Level::INFO, "Error recovery successful.");
    } catch (const franka::Exception& recovery_ex) {
        Logger::getInstance().log(Logger::Level::ERROR, "Error recovery failed: " + std::string(recovery_ex.what()));
    }
}

inline double filterStep(const double& update_frequency, double& filter_percentage) {
    const double kappa = -1 / (std::log(1 - std::min(filter_percentage, 0.999999)));
    return 1.0 / (kappa * update_frequency + 1.0);
}

template <typename T>
inline T filteredUpdate(T target, T current, double filter) {
    return (1.0 - filter) * current + filter * target;
}

void FrankaController::updateFilteredPose() {
    const double step = filterStep(update_frequency_, filter_params_pose_);
    position_d_ = filteredUpdate(position_d_target_, position_d_, step);
    orientation_d_ = orientation_d_.slerp(step, orientation_d_target_);
}

void FrankaController::updateFilteredStiffness() {
    if (filter_params_stiffness_ == 1.0) {
        // If no filtering is desired, directly assign target values
        cartesian_stiffness_ = cartesian_stiffness_target_;
        cartesian_damping_ = cartesian_damping_target_;
        nullspace_stiffness_ = nullspace_stiffness_;
        nullspace_damping_ = nullspace_damping_;
    } else {
        // Smoothly update the stiffness and damping values
        const double step = filterStep(update_frequency_, filter_params_stiffness_);

        cartesian_stiffness_ = filteredUpdate(cartesian_stiffness_target_, cartesian_stiffness_, step);
        cartesian_damping_ = filteredUpdate(cartesian_damping_target_, cartesian_damping_, step);
        nullspace_stiffness_ = filteredUpdate(nullspace_stiffness_, nullspace_stiffness_, step);
        nullspace_damping_ = filteredUpdate(nullspace_damping_, nullspace_damping_, step);
    }
}

void FrankaController::updateFilteredNullspaceConfig() {
    const double step = filterStep(update_frequency_, filter_params_nullspace_config_);
    q_d_nullspace_ = filteredUpdate(q_d_nullspace_target_, q_d_nullspace_, step);
}

void FrankaController::updateFilteredWrench() {
    const double step = filterStep(update_frequency_, filter_params_wrench_);
    cartesian_wrench_ = filteredUpdate(cartesian_wrench_target_, cartesian_wrench_, step);
}

void FrankaController::setFiltering(double update_frequency, double filter_params_nullspace_config, double filter_params_stiffness,
                                    double filter_params_pose, double filter_params_wrench) {
    update_frequency_ = update_frequency;
    filter_params_nullspace_config_ = filter_params_nullspace_config;
    filter_params_stiffness_ = filter_params_stiffness;
    filter_params_pose_ = filter_params_pose;
    filter_params_wrench_ = filter_params_wrench;
}

Eigen::VectorXd FrankaController::calculateCommandedTorques(const franka::RobotState& state, const franka::Model& model) {
    // Update filtered states
    updateFilteredNullspaceConfig();
    updateFilteredStiffness();
    updateFilteredPose();
    updateFilteredWrench();

    // Update Jacobian based on current state
    Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian_map(model.zeroJacobian(franka::Frame::kEndEffector, state).data());
    jacobian_ = jacobian_map;

    // Compute position and orientation errors
    Eigen::Vector3d position(state.O_T_EE[12], state.O_T_EE[13], state.O_T_EE[14]);
    Eigen::Quaterniond orientation(state.O_T_EE[0], state.O_T_EE[1], state.O_T_EE[2], state.O_T_EE[3]);
    Eigen::Vector3d position_error = position_d_ - position;
    Eigen::Vector3d orientation_error = calculateOrientationError(orientation_d_, orientation);

    Eigen::Matrix<double, 6, 1> error;
    error.head(3) = position_error;
    error.tail(3) = orientation_error;

    // Compute the pseudo-inverse of the Jacobian's transpose
    Eigen::MatrixXd jacobian_transpose_pinv;
    pseudoInverse(jacobian_.transpose(), &jacobian_transpose_pinv);

    // Compute torques for Cartesian impedance control
    Eigen::VectorXd tau_task = jacobian_.transpose() * (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian_ * Eigen::Map<const Eigen::Matrix<double, 7, 1>>(state.dq.data())));

    // Compute nullspace torques
    Eigen::VectorXd q_d_nullspace_eigen = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(q_d_nullspace_target_.data());
    Eigen::VectorXd q_eigen = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(state.q.data());
    Eigen::VectorXd tau_nullspace = (Eigen::MatrixXd::Identity(7, 7) - jacobian_.transpose() * jacobian_transpose_pinv) *
                                    (nullspace_stiffness_ * (q_d_nullspace_eigen - q_eigen) - nullspace_damping_ * Eigen::Map<const Eigen::Matrix<double, 7, 1>>(state.dq.data()));

    // Compute external wrench torques
    Eigen::VectorXd tau_ext = jacobian_.transpose() * cartesian_wrench_;

    // Combine torques
    Eigen::VectorXd tau_d = tau_task + tau_nullspace + tau_ext;

    // Saturate torque rate
    saturateTorqueRate(tau_d, &tau_c_, delta_tau_max_);

    return tau_c_;
}

void FrankaController::pseudoInverse(const Eigen::MatrixXd& matrix, Eigen::MatrixXd* pseudo_inverse) {
    double lambda_ = 0.2;  // Damping factor
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::VectorXd singular_values_inv = svd.singularValues();
    for (long i = 0; i < matrix.cols(); ++i) {
        singular_values_inv(i) = singular_values_inv(i) / (singular_values_inv(i) * singular_values_inv(i) + lambda_ * lambda_);
    }
    *pseudo_inverse = svd.matrixV() * singular_values_inv.asDiagonal() * svd.matrixU().transpose();
}

inline double saturateValue(double x, double x_min, double x_max) {
    return std::min(std::max(x, x_min), x_max);
}

void FrankaController::saturateTorqueRate(const Eigen::VectorXd& tau_d_calculated, Eigen::VectorXd* tau_d_saturated, double delta_tau_max) {
    for (int i = 0; i < tau_d_calculated.size(); ++i) {
        double difference = tau_d_calculated[i] - tau_d_saturated->operator()(i);
        tau_d_saturated->operator()(i) += saturateValue(difference, -delta_tau_max, delta_tau_max);
    }
}

void FrankaController::applyCartesianImpedanceControl(franka::Robot& robot, const franka::Model& model) {
    running_ = true;
    robot.control([this, &model](const franka::RobotState& state, franka::Duration duration) -> franka::Torques {
        time_ += duration.toSec();

        // Calculate the desired position and orientation
        Eigen::Vector3d position_d;
        Eigen::Quaterniond orientation_d;
        bool motion_finished = motion_generator_->generatePose(time_, &position_d, &orientation_d);

        if (motion_finished) {
            running_ = false;  // Stop the control loop when motion is finished
        }

        // Calculate commanded torques
        Eigen::VectorXd tau_d = calculateCommandedTorques(state, model);

        // Log the robot state to the file
        logRobotState(state, tau_d);

        std::cout << "O_T_EE_d[0]: " << state.O_T_EE_d[0]
          << " O_T_EE_d[1]: " << state.O_T_EE_d[1]
          << " O_T_EE_d[2]: " << state.O_T_EE_d[2] << std::endl;

        return std::array<double, 7>{tau_d[0], tau_d[1], tau_d[2], tau_d[3], tau_d[4], tau_d[5], tau_d[6]};
    });

    while (running_) {
        // Keep the control loop running until the movement phase is complete
        std::this_thread::sleep_for(std::chrono::milliseconds(10));  // Avoid busy-waiting
    }
}

void FrankaController::run() {
    Logger& logger = Logger::getInstance();
    logger.log(Logger::Level::INFO, "Starting FrankaController...");

    franka::Robot robot(robot_hostname_);
    franka::Model model = robot.loadModel();
    start_time_ = std::chrono::steady_clock::now();

    // Capture the robot's current (real) start pose
    franka::RobotState initial_state = robot.readOnce();
    real_start_pose_ = {
        initial_state.O_T_EE[12], initial_state.O_T_EE[13], initial_state.O_T_EE[14],
        initial_state.O_T_EE[3], initial_state.O_T_EE[0], initial_state.O_T_EE[1], initial_state.O_T_EE[2]
    };

    try {
        // Move to start pose
        std::cout << "Moving to start pose..." << std::endl;
        current_phase_ = MotionPhase::MOVE_TO_START;
        Eigen::Vector3d start_pos(start_pose_[0], start_pose_[1], start_pose_[2]);
        Eigen::Quaterniond start_ori(start_pose_[3], start_pose_[4], start_pose_[5], start_pose_[6]);
        motion_generator_ = std::make_unique<CartesianMotionGenerator>(speed_factor_, start_pos, start_ori);

        // Apply Cartesian impedance control to move to start pose
        applyCartesianImpedanceControl(robot, model);

        // After reaching start pose, move to end pose
        std::cout << "Moving to end pose..." << std::endl;
        current_phase_ = MotionPhase::MOVE_TO_END;
        Eigen::Vector3d end_pos(end_pose_[0], end_pose_[1], end_pose_[2]);
        Eigen::Quaterniond end_ori(end_pose_[3], end_pose_[4], end_pose_[5], end_pose_[6]);
        motion_generator_ = std::make_unique<CartesianMotionGenerator>(speed_factor_, end_pos, end_ori);

        // Apply Cartesian impedance control to move to end pose
        applyCartesianImpedanceControl(robot, model);

    } catch (const franka::Exception& ex) {
        running_ = false;
        logger.log(Logger::Level::ERROR, "Franka exception: " + std::string(ex.what()));
        performAutomaticErrorRecovery(robot);
    } catch (const std::exception& ex) {
        running_ = false;
        logger.log(Logger::Level::ERROR, "Standard exception: " + std::string(ex.what()));
    }

    // Ensure robot is stopped after the loop ends
    robot.stop();
}

