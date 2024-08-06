#include "robot_manager.h"

int main(int argc, char** argv) {
    const std::string config_file = "../params.json";
    try {
        RobotManager manager(config_file);
        manager.run();
    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << std::endl;
        return -1;
    }

    return 0;
}
