#include "franka_controller.h"
#include "motion_generator.h"

FrankaController::FrankaController(const nlohmann::json& params, const std::string& mq_name)
    : mq_name_(mq_name), mq_(boost::interprocess::open_or_create, mq_name.c_str(), 100, sizeof(char) * 256) {
    loadParameters(params);
    vel_current_ = 0.0;
    angle_ = 0.0;
    time_ = 0.0;
    running_ = true;

    rotation_matrix_ = Eigen::AngleAxisd(roll_, Eigen::Vector3d::UnitX())
                     * Eigen::AngleAxisd(pitch_, Eigen::Vector3d::UnitY())
                     * Eigen::AngleAxisd(yaw_, Eigen::Vector3d::UnitZ());

    log_file_path_ = "robot_log_" + std::to_string(time(nullptr)) + ".csv";
    initializeLogFile();
}

void FrankaController::loadParameters(const nlohmann::json& params) {
    robot_hostname_ = params["robot_hostname"];
    radius_ = params["radius"];
    vel_max_ = params["vel_max"];
    acceleration_time_ = params["acceleration_time"];
    run_time_ = params["run_time"];
    q_goal_ = params["q_goal"];
    k_gains_ = params["k_gains"];
    d_gains_ = params["d_gains"];
    roll_ = params["roll"];
    pitch_ = params["pitch"];
    yaw_ = params["yaw"];
}

void FrankaController::setDefaultBehavior(franka::Robot& robot) {
    robot.setCollisionBehavior(
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}
    );
    robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
    robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
}

void FrankaController::initializeLogFile() {
    log_file_.open(log_file_path_);
    if (!log_file_.is_open()) {
        throw std::runtime_error("Unable to open log file: " + log_file_path_);
    }
    log_file_ << "time,q[0],q[1],q[2],q[3],q[4],q[5],q[6],O_T_EE[0],O_T_EE[1],O_T_EE[2],O_T_EE[3],O_T_EE[4],O_T_EE[5],O_T_EE[6],O_T_EE[7],O_T_EE[8],O_T_EE[9],O_T_EE[10],O_T_EE[11],O_T_EE[12],O_T_EE[13],O_T_EE[14],O_T_EE[15],x,y,dx,dy\n";
}

void FrankaController::logRobotState(const franka::RobotState& robot_state, double x, double y, double dx, double dy) {
    log_file_ << robot_state.time.toMSec() << ","
              << robot_state.q[0] << "," << robot_state.q[1] << "," << robot_state.q[2] << "," 
              << robot_state.q[3] << "," << robot_state.q[4] << "," << robot_state.q[5] << "," 
              << robot_state.q[6] << ","
              << robot_state.O_T_EE[0] << "," << robot_state.O_T_EE[1] << "," << robot_state.O_T_EE[2] << "," 
              << robot_state.O_T_EE[3] << "," << robot_state.O_T_EE[4] << "," << robot_state.O_T_EE[5] << "," 
              << robot_state.O_T_EE[6] << "," << robot_state.O_T_EE[7] << "," << robot_state.O_T_EE[8] << "," 
              << robot_state.O_T_EE[9] << "," << robot_state.O_T_EE[10] << "," << robot_state.O_T_EE[11] << "," 
              << robot_state.O_T_EE[12] << "," << robot_state.O_T_EE[13] << "," << robot_state.O_T_EE[14] << "," 
              << robot_state.O_T_EE[15] << ","
              << x << "," << y << "," << dx << "," << dy << "\n";
}

void FrankaController::sendMessage(const std::string& message) {
    mq_.send(message.data(), message.size(), 0);
}

std::string FrankaController::receiveMessage() {
    char buffer[256];
    std::size_t received_size;
    unsigned int priority;
    mq_.receive(buffer, sizeof(buffer), received_size, priority);
    return std::string(buffer, received_size);
}

void FrankaController::run() {
    try {
        franka::Robot robot(robot_hostname_);
        setDefaultBehavior(robot);

        MotionGenerator motion_generator(0.5, q_goal_);
        std::cout << "WARNING: This example will move the robot! "
                  << "Please make sure to have the user stop button at hand!" << std::endl
                  << "Press Enter to continue..." << std::endl;
        std::cin.ignore();
        robot.control(motion_generator);
        std::cout << "Finished moving to initial joint configuration." << std::endl;

        robot.setCollisionBehavior(
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}
        );

        model_ = std::make_unique<franka::Model>(robot.loadModel());

        robot.control(
            [this](const franka::RobotState& robot_state, franka::Duration period) -> franka::Torques {
                return this->impedanceControlCallback(robot_state, period);
            },
            [this](const franka::RobotState& robot_state, franka::Duration period) -> franka::CartesianPose {
                return this->cartesianPoseCallback(robot_state, period);
            }
        );
    } catch (const franka::Exception& ex) {
        running_ = false;
        std::cerr << ex.what() << std::endl;
    }
}

franka::CartesianPose FrankaController::cartesianPoseCallback(const franka::RobotState& robot_state, franka::Duration period) {
    time_ += period.toSec();
    if (time_ == 0.0) {
        initial_pose_ = robot_state.O_T_EE_c;
    }

    if (vel_current_ < vel_max_ && time_ < run_time_) {
        vel_current_ += period.toSec() * std::fabs(vel_max_ / acceleration_time_);
    }
    if (vel_current_ > 0.0 && time_ > run_time_) {
        vel_current_ -= period.toSec() * std::fabs(vel_max_ / acceleration_time_);
    }
    vel_current_ = std::fmax(vel_current_, 0.0);
    vel_current_ = std::fmin(vel_current_, vel_max_);

    angle_ += period.toSec() * vel_current_ / std::fabs(radius_);
    if (angle_ > 2 * M_PI) {
        angle_ -= 2 * M_PI;
    }

    double delta_x = radius_ * (1 - std::cos(angle_));
    double delta_y = radius_ * std::sin(angle_);
    Eigen::Vector3d delta_position(delta_x, delta_y, 0.0);
    Eigen::Vector3d rotated_delta_position = rotation_matrix_ * delta_position;

    franka::CartesianPose pose_desired = initial_pose_;
    pose_desired.O_T_EE[12] += rotated_delta_position[0];
    pose_desired.O_T_EE[13] += rotated_delta_position[1];
    pose_desired.O_T_EE[14] += rotated_delta_position[2];

    if (time_ >= run_time_ + acceleration_time_) {
        running_ = false;
        return franka::MotionFinished(pose_desired);
    }

    // Log the robot state
    logRobotState(robot_state, delta_position[0], delta_position[1], vel_current_ * cos(angle_), vel_current_ * sin(angle_));

    // Send robot state via message queue
    std::ostringstream oss;
    oss << "State: " << robot_state.q[0] << "," << robot_state.q[1] << "," << robot_state.q[2] << ","
        << robot_state.q[3] << "," << robot_state.q[4] << "," << robot_state.q[5] << "," << robot_state.q[6];
    sendMessage(oss.str());

    return pose_desired;
}

franka::Torques FrankaController::impedanceControlCallback(const franka::RobotState& state, franka::Duration /*period*/) {
    std::array<double, 7> coriolis = model_->coriolis(state);
    std::array<double, 7> tau_d_calculated;
    for (size_t i = 0; i < 7; i++) {
        tau_d_calculated[i] =
            k_gains_[i] * (state.q_d[i] - state.q[i]) - d_gains_[i] * state.dq[i] + coriolis[i];
    }
    std::array<double, 7> tau_d_rate_limited =
        franka::limitRate(franka::kMaxTorqueRate, tau_d_calculated, state.tau_J_d);
    return tau_d_rate_limited;
}
