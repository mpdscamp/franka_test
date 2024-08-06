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
#include "motion_generator.h"

class FrankaController {
public:
    FrankaController(const nlohmann::json& params, const std::string& mq_name);
    void run();

private:
    void loadParameters(const nlohmann::json& params);
    franka::CartesianPose cartesianPoseCallback(const franka::RobotState& robot_state, franka::Duration period);
    franka::Torques impedanceControlCallback(const franka::RobotState& state, franka::Duration period);
    void setDefaultBehavior(franka::Robot& robot);
    void initializeLogFile();
    void logRobotState(const franka::RobotState& robot_state, double x, double y, double dx, double dy);
    void sendMessage(const std::string& message);
    std::string receiveMessage();

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

    double vel_current_;
    double angle_;
    double time_;
    std::atomic_bool running_;
    Eigen::Matrix3d rotation_matrix_;
    std::array<double, 16> initial_pose_;
    std::unique_ptr<franka::Model> model_;

    std::ofstream log_file_;
    std::string log_file_path_;

    std::string mq_name_;
    boost::interprocess::message_queue mq_;
};

#endif // FRANKA_CONTROLLER_H_
