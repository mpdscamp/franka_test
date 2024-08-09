#ifndef ROBOT_LOGGER_H_
#define ROBOT_LOGGER_H_

#include <iostream>
#include <sstream>
#include <string>

class RobotLogger {
public:
    RobotLogger(const std::string& robot_name, std::ostream& os = std::cout);

    template <typename T>
    RobotLogger& operator<<(const T& msg);

    RobotLogger& operator<<(std::ostream& (*manip)(std::ostream&));

private:
    std::string robot_name_;
    std::ostream& os_;
    std::ostringstream buffer_;  // Buffer to accumulate the message
};

// Template function definition must be in the header file
template <typename T>
RobotLogger& RobotLogger::operator<<(const T& msg) {
    buffer_ << msg;  // Accumulate the message in the buffer
    return *this;
}

#endif // ROBOT_LOGGER_H_
