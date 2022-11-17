cmake_minimum_required(VERSION 3.5)
project(GRFcontroller LANGUAGES CXX)

find_package(Eigen3 REQUIRED HINTS ${Eigen3_HINT})
find_package(camel-tools-trajectory REQUIRED)
find_package(camel-tools-thread REQUIRED)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${marten-leg_util_SOURCE_DIR}/include
)

add_library(${PROJECT_NAME} STATIC
        src/GRFcontroller.cpp
        )

add_library(marten-leg_controller::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}
        camel-tools-trajectory
        marten-leg_util
        camel-tools-thread
        )