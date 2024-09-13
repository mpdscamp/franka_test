#ifndef ROBOT_MANAGER_H_
#define ROBOT_MANAGER_H_

#include <dirent.h>
#include <sys/types.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include <boost/interprocess/ipc/message_queue.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <thread>
#include <algorithm>
#include "utils/logger.h"
#include "control/franka_controller.h"

class RobotManager {
public:
    RobotManager(const std::string& config_file);
    void run();

private:
    void loadConfig(const std::string& config_file);
    void createRobotProcess(const nlohmann::json& robot_parameters, const std::string& mq_name, int robot_id);

    std::vector<nlohmann::json> robots_parameters_;
    std::string central_mq_name_;  // Centralized message queue name

    bool running_ = true;
};

#endif // ROBOT_MANAGER_H_
