#include "communication/central_communication_hub.h"

CentralCommunicationHub::CentralCommunicationHub(const std::string& mq_name)
    : mq_name_(mq_name), mq_(boost::interprocess::open_or_create, mq_name.c_str(), 100, sizeof(char) * 256) {}

void CentralCommunicationHub::sendState(const std::string& state_message, int robot_id) {
    std::string tagged_message = std::to_string(robot_id) + ":" + state_message;
    mq_.send(tagged_message.data(), tagged_message.size(), 0);
    Logger::getInstance().log(Logger::Level::DEBUG, "Sent state message: " + tagged_message);
}

std::string CentralCommunicationHub::receiveState(int robot_id) {
    char buffer[256];
    std::size_t received_size;
    unsigned int priority;
    mq_.receive(buffer, sizeof(buffer), received_size, priority);
    std::string received_message(buffer, received_size);

    int sender_robot_id = std::stoi(received_message.substr(0, received_message.find(':')));
    std::string message_content = received_message.substr(received_message.find(':') + 1);

    Logger::getInstance().log(Logger::Level::DEBUG, "Received state message from robot " + std::to_string(sender_robot_id) + ": " + message_content);

    return message_content;
}

void CentralCommunicationHub::receiveAndStoreState(int robot_id) {
    char buffer[256];
    std::size_t received_size;
    unsigned int priority;

    while (mq_.try_receive(buffer, sizeof(buffer), received_size, priority)) {
        std::string received_message(buffer, received_size);
        int sender_robot_id = std::stoi(received_message.substr(0, received_message.find(':')));
        std::string message_content = received_message.substr(received_message.find(':') + 1);

        // Deserialize and store the state
        franka::RobotState state = StateSerializer::deserialize(message_content);
        states_[sender_robot_id] = state;
        last_update_time_[sender_robot_id] = std::chrono::steady_clock::now();

        Logger::getInstance().log(Logger::Level::DEBUG, "Stored state from robot " + std::to_string(sender_robot_id));
    }
}

void CentralCommunicationHub::broadcastStates() {
    Logger::getInstance().log(Logger::Level::INFO, "Broadcasting robot states.");

    for (const auto& [sender_robot_id, state] : states_) {
        std::string state_message = StateSerializer::serialize(state);
        for (int target_robot_id = 1; target_robot_id <= states_.size(); ++target_robot_id) {
            if (target_robot_id != sender_robot_id) {
                sendState(state_message, target_robot_id);

                // Log the broadcast event
                Logger::getInstance().log(Logger::Level::DEBUG,
                    "Robot " + std::to_string(target_robot_id) + " received state from Robot " + std::to_string(sender_robot_id));
            }
        }
    }
}

franka::RobotState CentralCommunicationHub::getState(int robot_id) const {
    auto it = states_.find(robot_id);
    if (it != states_.end()) {
        return it->second;
    } else {
        throw std::runtime_error("Robot ID " + std::to_string(robot_id) + " not found in central communication hub");
    }
}

bool CentralCommunicationHub::isStateRecent(int robot_id, std::chrono::milliseconds timeout) const {
    auto it = last_update_time_.find(robot_id);
    if (it != last_update_time_.end()) {
        auto now = std::chrono::steady_clock::now();
        return (now - it->second) <= timeout;
    } else {
        return false;
    }
}

const std::map<int, franka::RobotState>& CentralCommunicationHub::getAllStates() const {
    return states_;
}

