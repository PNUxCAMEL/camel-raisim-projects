cmake_minimum_required(VERSION 3.5)
project(ControlUtils LANGUAGES CXX)

find_package(Eigen3 REQUIRED HINTS ${Eigen3_HINT})

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${marten-leg_util_SOURCE_DIR}/include
)

add_library(${PROJECT_NAME} STATIC
        src/GRFEstimatorMLP.cpp
        src/GRFEstimatorSMO.cpp
        src/GRFEstimatorETO.cpp)

add_library(marten-leg_controller::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}
        tensorflow
        marten-leg_util)