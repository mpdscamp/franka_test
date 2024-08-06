cmake_minimum_required(VERSION 3.10)
project(FRANKA_TEST LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

# Find required packages
find_package(Eigen3 REQUIRED)
set(Franka_DIR ${CMAKE_SOURCE_DIR}/../libfranka/build)
find_package(Franka REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)

# Fetch nlohmann_json
include(FetchContent)
FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.11.2/json.tar.xz)
FetchContent_MakeAvailable(json)

include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}
    ${Boost_INCLUDE_DIRS}
)

# Add the executable
add_executable(main src/main.cpp src/franka_controller.cpp src/motion_generator.cpp src/robot_manager.cpp)

# Link the libraries
target_link_libraries(main Eigen3::Eigen Franka::Franka nlohmann_json::nlohmann_json pthread ${Boost_LIBRARIES} rt)
