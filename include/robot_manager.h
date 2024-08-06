#ifndef ROBOT_MANAGER_H_
#define ROBOT_MANAGER_H_

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include <boost/interprocess/ipc/message_queue.hpp>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <thread>
#include <algorithm>

class RobotManager {
public:
    RobotManager(const std::string& config_file);
    void run();

private:
    void loadConfig(const std::string& config_file);
    void createRobotProcess(const nlohmann::json& robot_params, const std::string& mq_name);
    void listenToMessages(const std::string& mq_name);

    std::vector<nlohmann::json> robots_params_;
    std::vector<std::string> mq_names_;
};

#endif // ROBOT_MANAGER_H_
