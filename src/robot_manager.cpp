#include "robot_manager.h"
#include "franka_controller.h"
#include "robot_logger.h"

RobotManager::RobotManager(const std::string& config_file)
    : logger_("RobotManager") {  // Initialize the logger with "RobotManager" as the name
    loadConfig(config_file);
}

void RobotManager::loadConfig(const std::string& config_file) {
    std::ifstream json_file(config_file);
    if (!json_file) {
        throw std::runtime_error("Unable to open JSON file: " + config_file);
    }

    nlohmann::json config;
    json_file >> config;
    robots_params_ = config["robots"];

    for (size_t i = 0; i < robots_params_.size(); ++i) {
        mq_names_.emplace_back("mq_robot_" + std::to_string(i));
    }
}

void RobotManager::createRobotProcess(const nlohmann::json& robot_params, const std::string& mq_name, const std::vector<std::string>& peer_mq_names) {
    pid_t pid = fork();
    if (pid == 0) {
        // Child process
        try {
            FrankaController controller(robot_params, mq_name, peer_mq_names);  // Pass the list of peer_mq_names for multi-robot communication
            controller.run();
            logger_ << "Robot process finished" << std::endl;
        } catch (const std::exception& ex) {
            logger_ << "Error: " << ex.what() << std::endl;
        }
        exit(0);
    } else if (pid > 0) {
        // Parent process
        logger_ << "Created child process with PID: " << pid << std::endl;
        return;
    } else {
        // Fork failed
        throw std::runtime_error("Fork failed!");
    }
}

void RobotManager::listenToMessages(const std::string& mq_name) {
    boost::interprocess::message_queue mq(boost::interprocess::open_or_create, mq_name.c_str(), 100, sizeof(char) * 256);
    char buffer[256];
    std::size_t received_size;
    unsigned int priority;

    while (running_) {  // running_ is a flag to indicate whether to keep running
        // Use a timed receive to allow the thread to check the running_ flag periodically
        bool received = mq.timed_receive(buffer, sizeof(buffer), received_size, priority, boost::posix_time::microsec_clock::universal_time() + boost::posix_time::milliseconds(100));
         
        if (received) {
            std::string message(buffer, received_size);
            logger_ << "Received from " << mq_name << ": " << message << std::endl;
            // Process message here
        }
    }

    logger_ << "Listener for " << mq_name << " exiting." << std::endl;
}

void RobotManager::run() {
    // Create robot processes with the list of peer message queue names
    for (size_t i = 0; i < robots_params_.size(); ++i) {
        std::vector<std::string> peer_mq_names;
        for (size_t j = 0; j < mq_names_.size(); ++j) {
            if (j != i) {
                peer_mq_names.push_back(mq_names_[j]);
            }
        }
        createRobotProcess(robots_params_[i], mq_names_[i], peer_mq_names);
    }

    std::vector<std::thread> listener_threads;
    running_ = true;  // Ensure running_ is true when starting

    // Start listener threads for each robot's message queue
    for (const auto& mq_name : mq_names_) {
        listener_threads.emplace_back(&RobotManager::listenToMessages, this, mq_name);
    }

    // Wait for all child processes (robot processes) to finish
    for (size_t i = 0; i < robots_params_.size(); ++i) {
        int status;
        pid_t pid = wait(&status);
        if (pid != -1) {
            logger_ << "Child process with PID " << pid << " has finished." << std::endl;
        } else {
            logger_ << "Error while waiting for child processes." << std::endl;
        }
    }

    // Signal listener threads to stop listening
    running_ = false;

    // Join listener threads
    for (auto& thread : listener_threads) {
        if (thread.joinable()) {
            thread.join();
        }
    }

    logger_ << "All robot processes have finished. Exiting program." << std::endl;

    // Kill the parent process to end the program
    exit(0);
}
