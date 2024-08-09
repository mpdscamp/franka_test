#ifndef FRANKA_CONTROLLER_H_
#define FRANKA_CONTROLLER_H_

#include <franka/robot.h>
#include <franka/model.h>
#include <nlohmann/json.hpp>
#include <Eigen/Dense>
#include <atomic>
#include <array>
#include <fstream>
#include <iostream>
#include <string>
#include <boost/interprocess/ipc/message_queue.hpp>
#include <franka/exception.h>
#include <franka/rate_limiting.h>
#include <thread>
#include "robot_logger.h"

class FrankaController {
public:
    FrankaController(const nlohmann::json& params, const std::string& mq_name, const std::vector<std::string>& peer_mq_names);
    void run();

private:
    void loadParameters(const nlohmann::json& params);
    void setDefaultBehavior(franka::Robot& robot);
    void sendMessage(const std::string& message);
    std::string receiveMessage();
    void printCurrentQVector(franka::Robot& robot);
    void sendState(franka::Robot& robot);
    void performAutomaticErrorRecovery(franka::Robot& robot);
    franka::RobotState receiveState();
    std::string serializeState(const franka::RobotState& state);
    franka::RobotState deserializeState(const std::string& state_message);
    void receiveAndProcessPeerStates();

    std::string robot_hostname_;
    double radius_;
    double vel_max_;
    double acceleration_time_;
    double run_time_;
    std::array<double, 7> q_goal_;
    std::array<double, 7> k_gains_;
    std::array<double, 7> d_gains_;
    double roll_;
    double pitch_;
    double yaw_;

    std::array<double, 7> dq_max_;
    std::array<double, 7> ddq_max_start_;
    std::array<double, 7> ddq_max_goal_;

    double initial_angle_;

    double vel_current_;
    double angle_;
    double time_;
    std::atomic_bool running_;
    Eigen::Matrix3d rotation_matrix_;
    std::array<double, 16> initial_pose_;
    std::array<double, 16> border_pose_array_;
    Eigen::Vector3d center_;
    std::unique_ptr<franka::Model> model_;

    std::string mq_name_;
    std::string peer_mq_name_;
    boost::interprocess::message_queue mq_;
    std::vector<std::unique_ptr<boost::interprocess::message_queue>> peer_mqs_;  // Change to vector of unique_ptr

    RobotLogger logger_;

    // Global state management
    struct GlobalState {
        std::map<int, franka::RobotState> states;  // Map of robot ID to their state
        std::map<int, std::chrono::steady_clock::time_point> last_update_time; // Map to store the last update time of each robot

        void updateState(int robot_id, const franka::RobotState& state) {
            states[robot_id] = state;
            last_update_time[robot_id] = std::chrono::steady_clock::now();
        }

        franka::RobotState getState(int robot_id) const {
            auto it = states.find(robot_id);
            if (it != states.end()) {
                return it->second;
            } else {
                throw std::runtime_error("Robot ID not found in global state map");
            }
        }

        bool isStateRecent(int robot_id, std::chrono::milliseconds timeout) const {
            auto now = std::chrono::steady_clock::now();
            return (now - last_update_time.at(robot_id)) <= timeout;
        }
    };

    GlobalState global_state;
};

#endif // FRANKA_CONTROLLER_H_