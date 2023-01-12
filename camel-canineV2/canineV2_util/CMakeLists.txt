cmake_minimum_required(VERSION 3.5)
project(canineV2_util LANGUAGES CXX)

find_package(camel-tools-thread)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
)

add_library(canineV2_util
        SHARED
        src/CanMotorFL.cpp
        src/CanMotorFR.cpp
        src/CanMotorRL.cpp
        src/CanMotorRR.cpp
        src/Command.cpp
        src/StateEstimator.cpp
        src/RobotMath.cpp
        src/Filter.cpp
        src/XboxController.cpp
        src/XboxCommand.cpp)
target_link_libraries(canineV2_util
        PUBLIC
        raisim::raisim
        PRIVATE
        pthread
        camel-tools-thread
)