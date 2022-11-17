cmake_minimum_required(VERSION 3.5)
project(ControlUtils LANGUAGES CXX)

find_package(Eigen3 REQUIRED HINTS ${Eigen3_HINT})

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${canine-leg-left_util_SOURCE_DIR}/include
)

add_library(${PROJECT_NAME} STATIC
        src/GRFEstimatorMLP.cpp
        src/GRFEstimatorSMO.cpp
        src/GRFEstimatorETO.cpp include/ControlUtils/GRFEstimatorETO.hpp)

add_library(canine-leg-left_controller::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}
        tensorflow
        canine-leg-left_util)