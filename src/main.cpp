#include "control/robot_manager.h"
#include "utils/logger.h"

int main(int argc, char** argv) {
    const std::string config_file = "../parameters.json";  // Ensure this path is correct

    try {
        Logger& logger = Logger::getInstance();
        logger.setLogLevel(Logger::Level::DEBUG);

        logger.log(Logger::Level::INFO, "Program started");

        RobotManager manager(config_file);
        manager.run();

    } catch (const std::exception& ex) {
        Logger::getInstance().log(Logger::Level::ERROR, std::string("Error: ") + ex.what());
        return -1;
    }

    return 0;
}
