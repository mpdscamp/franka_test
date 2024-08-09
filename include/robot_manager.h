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
#include "robot_logger.h"

class RobotManager {
public:
    RobotManager(const std::string& config_file);
    void run();

private:
    void loadConfig(const std::string& config_file);
    // Update the function signature here to accept a vector of strings for peer message queue names
    void createRobotProcess(const nlohmann::json& robot_params, const std::string& mq_name, const std::vector<std::string>& peer_mq_names);
    void listenToMessages(const std::string& mq_name);

    std::vector<nlohmann::json> robots_params_;
    std::vector<std::string> mq_names_;

    bool running_ = true;

    RobotLogger logger_;
};

#endif // ROBOT_MANAGER_H_
