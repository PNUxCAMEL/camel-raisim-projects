cmake_minimum_required(VERSION 3.5)
project(canine_util LANGUAGES CXX)

find_package(camel-tools-thread)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
)

add_library(canine_util
        SHARED
        src/CanMotorBackward.cpp
        src/CanMotorForward.cpp
        src/Command.cpp
        src/StateEstimator.cpp)
target_link_libraries(canine_util
        PUBLIC
        raisim::raisim
        PRIVATE
        pthread
        camel-tools-thread
)