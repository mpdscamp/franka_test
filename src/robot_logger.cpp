#include "robot_logger.h"

RobotLogger::RobotLogger(const std::string& robot_name, std::ostream& os)
    : robot_name_(robot_name), os_(os) {}

RobotLogger& RobotLogger::operator<<(std::ostream& (*manip)(std::ostream&)) {
    os_ << "[" << robot_name_ << "] " << buffer_.str();  // Prepend the robot name and output the buffer
    buffer_.str("");  // Clear the buffer
    os_ << manip;  // Apply the manipulator (e.g., std::endl)
    return *this;
}
