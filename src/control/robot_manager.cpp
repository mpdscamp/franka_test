#include "control/robot_manager.h"

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
    robots_parameters_ = config["robots"];

    central_mq_name_ = "central_mq";  // Use a single centralized message queue
}

void RobotManager::createRobotProcess(const nlohmann::json& robot_parameters, const std::string& mq_name, int robot_id) {
    pid_t pid = fork();
    if (pid == 0) {
        // Child process
        try {
            FrankaController controller(robot_parameters, mq_name, robot_id);  // Pass the central MQ name and the robot ID
            controller.run();
            Logger::getInstance().log(Logger::Level::INFO, "Robot process finished");
        } catch (const std::exception& ex) {
            Logger::getInstance().log(Logger::Level::ERROR, "Error: " + std::string(ex.what()));
        }
        exit(0);
    } else if (pid > 0) {
        // Parent process
        Logger::getInstance().log(Logger::Level::INFO, "Created child process with PID: " + std::to_string(pid));
        return;
    } else {
        // Fork failed
        throw std::runtime_error("Fork failed!");
    }
}

void RobotManager::run() {
    int robot_id = 1;
    for (const auto& robot_parameters : robots_parameters_) {
        createRobotProcess(robot_parameters, central_mq_name_, robot_id);
        robot_id++;
    }

    for (size_t i = 0; i < robots_parameters_.size(); ++i) {
        int status;
        pid_t pid = wait(&status);
        if (pid != -1) {
            Logger::getInstance().log(Logger::Level::INFO, "Child process with PID " + std::to_string(pid) + " has finished.");
        } else {
            Logger::getInstance().log(Logger::Level::ERROR, "Error while waiting for child processes.");
        }
    }

    Logger::getInstance().log(Logger::Level::INFO, "All robot processes have finished. Exiting program.");

    exit(0);
}
