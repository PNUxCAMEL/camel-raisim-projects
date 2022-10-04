cmake_minimum_required(VERSION 3.5)
project(PDcontroller LANGUAGES CXX)

find_package(camel-tools-trajectory REQUIRED)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${canine_util_SOURCE_DIR}/include
)

add_library(${PROJECT_NAME} STATIC
        src/JointPDController.cpp
        src/BezierTrajectoryGenerator.cpp
)

add_library(canine_controller::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}
        camel-tools-trajectory
        raisim::raisim
        canine_util
)