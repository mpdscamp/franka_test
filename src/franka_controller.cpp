#include "franka_controller.h"
#include "motion_generator.h"

template <typename T, std::size_t N>
std::ostream& operator<<(std::ostream& os, const std::array<T, N>& array) {
    os << "[";
    for (std::size_t i = 0; i < N; ++i) {
        os << array[i];
        if (i != N - 1) {
            os << ", ";
        }
    }
    os << "]";
    return os;
}

FrankaController::FrankaController(const nlohmann::json& params, const std::string& mq_name, const std::vector<std::string>& peer_mq_names)
    : mq_name_(mq_name),
      logger_(mq_name_),  // Initialize the logger with the robot's name
      mq_(boost::interprocess::open_or_create, mq_name.c_str(), 100, sizeof(char) * 256) {
    // Initialize peer message queues
    for (const auto& peer_mq_name : peer_mq_names) {
        peer_mqs_.emplace_back(std::make_unique<boost::interprocess::message_queue>(
            boost::interprocess::open_or_create, peer_mq_name.c_str(), 100, sizeof(char) * 256));
    }

    loadParameters(params);
    vel_current_ = 0.0;
    angle_ = 0.0;
    time_ = 0.0;
    running_ = true;

    rotation_matrix_ = Eigen::AngleAxisd(roll_, Eigen::Vector3d::UnitX())
                     * Eigen::AngleAxisd(pitch_, Eigen::Vector3d::UnitY())
                     * Eigen::AngleAxisd(yaw_, Eigen::Vector3d::UnitZ());
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
    dq_max_ = params["dq_max"];
    ddq_max_start_ = params["ddq_max_start"];
    ddq_max_goal_ = params["ddq_max_goal"];
    initial_angle_ = params["initial_angle"];
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

void FrankaController::printCurrentQVector(franka::Robot& robot) {
    // Read the current state of the robot
    franka::RobotState robot_state = robot.readOnce();
    
    // Access the q vector (joint positions)
    std::array<double, 7> q = robot_state.q;

    // Print the q vector
    std::cout << "Current q vector: [";
    for (size_t i = 0; i < q.size(); ++i) {
        std::cout << q[i];
        if (i != q.size() - 1) {
            std::cout << ", ";
        }
    }
    std::cout << "]" << std::endl;
}

void FrankaController::sendState(franka::Robot& robot) {
    franka::RobotState state = robot.readOnce();
    std::string state_message = serializeState(state);
    mq_.send(state_message.data(), state_message.size(), 0);
}

franka::RobotState FrankaController::receiveState() {
    char buffer[256];
    std::size_t received_size;
    unsigned int priority;
    mq_.receive(buffer, sizeof(buffer), received_size, priority);
    return deserializeState(std::string(buffer, received_size));
}

std::string FrankaController::serializeState(const franka::RobotState& state) {
    // Serialize the RobotState into a string (simple example)
    std::ostringstream oss;
    for (double q : state.q) {
        oss << q << " ";
    }
    return oss.str();
}

franka::RobotState FrankaController::deserializeState(const std::string& state_message) {
    // Deserialize the string back into a RobotState (simple example)
    franka::RobotState state;
    std::istringstream iss(state_message);
    for (double& q : state.q) {
        iss >> q;
    }
    return state;
}

void FrankaController::performAutomaticErrorRecovery(franka::Robot& robot) {
    try {
        robot.automaticErrorRecovery();
        std::cerr << "Error recovery successful." << std::endl;
    } catch (const franka::Exception& recovery_ex) {
        std::cerr << "Error recovery failed: " << recovery_ex.what() << std::endl;
    }
}

void FrankaController::receiveAndProcessPeerStates() {
    for (size_t i = 0; i < peer_mqs_.size(); ++i) {
        char buffer[256];
        std::size_t received_size;
        unsigned int priority;

        if (peer_mqs_[i]->try_receive(buffer, sizeof(buffer), received_size, priority)) {  // Dereference unique_ptr
            std::string received_message(buffer, received_size);
            franka::RobotState received_state = deserializeState(received_message);

            // Safely update the global state
            global_state.updateState(i, received_state);

            logger_ << "Received state from Robot " << i << ": ";
            for (double q : received_state.q) {
                logger_ << q << " ";
            }
            logger_ << std::endl;
        }
    }
}

void FrankaController::run() {
    for (size_t i = 0; i < peer_mqs_.size(); ++i) {
        global_state.updateState(i, franka::RobotState{});  // Initialize with default state
    }

    franka::Robot robot(robot_hostname_);

    try {
        setDefaultBehavior(robot);

        // Initial state check and recovery
        try {
            franka::RobotState robot_state = robot.readOnce();
            for (size_t i = 0; i < robot_state.q.size(); ++i) {
                if (!std::isfinite(robot_state.q[i])) {
                    throw std::runtime_error("Robot joint state contains NaN or infinite values.");
                }
            }
            logger_ << "Robot is in a good state. Proceeding with operation." << std::endl;
        } catch (const std::exception& ex) {
            logger_ << "Robot is in a bad state: " << ex.what() << std::endl;
            performAutomaticErrorRecovery(robot);
        }

        MotionGenerator motion_generator(0.5, q_goal_, dq_max_, ddq_max_start_, ddq_max_goal_);

        bool motion_success = false;
        int attempts = 0;
        const int max_attempts = 5;

        while (!motion_success && attempts < max_attempts) {
            try {
                logger_ << "Controlling the robot to the initial position (Attempt " << attempts + 1 << ")..." << std::endl;
                robot.control(motion_generator);
                logger_ << "Finished moving to initial joint configuration." << std::endl;
                motion_success = true;
            } catch (const std::exception& ex) {
                logger_ << "Error during initial motion: " << ex.what() << std::endl;
                logger_ << "Attempting automatic error recovery..." << std::endl;
                performAutomaticErrorRecovery(robot);
                attempts++;
            }
        }

        if (!motion_success) {
            logger_ << "Failed to move to initial position after " << max_attempts << " attempts." << std::endl;
            return;
        }

        auto last_send_time = std::chrono::steady_clock::now();

        while (running_) {
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_send_time).count();

            if (elapsed_time >= 500) {
                logger_ << "Sending state to peer: " << mq_name_ << " -> " << peer_mq_name_ << std::endl;
                sendState(robot);

                receiveAndProcessPeerStates(); // Receive and process states from peers

                last_send_time = current_time;
            }

            // Synchronization logic example:
            bool all_good = true;
            for (int i = 0; i < peer_mqs_.size(); ++i) {
                if (!global_state.isStateRecent(i, std::chrono::milliseconds(1000))) {
                    all_good = false;
                    break;
                }
            }

            if (all_good) {
                logger_ << "All robots are synchronized and in good state. Proceeding to the next step." << std::endl;
                // Add synchronization-specific logic here
            }

            std::this_thread::sleep_for(std::chrono::microseconds(10));
        }

    } catch (const franka::Exception& ex) {
        running_ = false;
        logger_ << "Franka exception: " << ex.what() << std::endl;
        performAutomaticErrorRecovery(robot);
    } catch (const std::exception& ex) {
        running_ = false;
        logger_ << "Standard exception: " << ex.what() << std::endl;
    }
}