#include "robot_manager.h"
#include "franka_controller.h"

RobotManager::RobotManager(const std::string& config_file) {
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

void RobotManager::createRobotProcess(const nlohmann::json& robot_params, const std::string& mq_name) {
    pid_t pid = fork();
    if (pid == 0) {
        // Child process
        try {
            FrankaController controller(robot_params, mq_name);
            controller.run();
        } catch (const std::exception& ex) {
            std::cerr << "Error: " << ex.what() << std::endl;
        }
        exit(0);
    } else if (pid > 0) {
        // Parent process
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
    while (true) {
        mq.receive(buffer, sizeof(buffer), received_size, priority);
        std::string message(buffer, received_size);
        std::cout << "Received from " << mq_name << ": " << message << std::endl;
        // Process message here
    }
}

void RobotManager::run() {
    for (size_t i = 0; i < robots_params_.size(); ++i) {
        createRobotProcess(robots_params_[i], mq_names_[i]);
    }

    std::vector<std::thread> listener_threads;

    for (const auto& mq_name : mq_names_) {
        listener_threads.emplace_back(&RobotManager::listenToMessages, this, mq_name);
    }

    for (auto& thread : listener_threads) {
        thread.join();
    }

    for (size_t i = 0; i < robots_params_.size(); ++i) {
        int status;
        wait(&status); // Wait for child processes to finish
    }
}
