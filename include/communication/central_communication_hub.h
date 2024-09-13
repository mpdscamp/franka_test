#ifndef CENTRAL_COMMUNICATION_HUB_H_
#define CENTRAL_COMMUNICATION_HUB_H_

#include <franka/robot_state.h>
#include <boost/interprocess/ipc/message_queue.hpp>
#include <map>
#include <string>
#include <chrono>
#include <stdexcept>
#include "utils/logger.h"
#include "utils/state_serializer.h"

class CentralCommunicationHub {
public:
    CentralCommunicationHub(const std::string& mq_name);

    // Methods to send and receive state messages
    void sendState(const std::string& state_message, int robot_id);
    std::string receiveState(int robot_id);

    // Method to store and broadcast states
    void receiveAndStoreState(int robot_id);
    void broadcastStates();

    // Methods to get state information
    franka::RobotState getState(int robot_id) const;
    bool isStateRecent(int robot_id, std::chrono::milliseconds timeout) const;

    // Utility to get all states, if needed
    const std::map<int, franka::RobotState>& getAllStates() const;

private:
    std::string mq_name_;
    boost::interprocess::message_queue mq_;

    // Maps to store the state and time of the last update
    std::map<int, franka::RobotState> states_;
    std::map<int, std::chrono::steady_clock::time_point> last_update_time_;
};

#endif // CENTRAL_COMMUNICATION_HUB_H_
