#ifndef STATE_SERIALIZER_H_
#define STATE_SERIALIZER_H_

#include <franka/robot_state.h>
#include <string>
#include <sstream>

class StateSerializer {
public:
    // Serialize the robot state into a string
    static std::string serialize(const franka::RobotState& state);

    // Deserialize the string back into a robot state
    static franka::RobotState deserialize(const std::string& state_message);
};

#endif // STATE_SERIALIZER_H_
