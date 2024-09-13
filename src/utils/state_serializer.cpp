#include "utils/state_serializer.h"

std::string StateSerializer::serialize(const franka::RobotState& state) {
    std::ostringstream oss;
    for (double q : state.q) {
        oss << q << " ";
    }
    return oss.str();
}

franka::RobotState StateSerializer::deserialize(const std::string& state_message) {
    franka::RobotState state;
    std::istringstream iss(state_message);
    for (double& q : state.q) {
        iss >> q;
    }
    return state;
}
