cmake_minimum_required(VERSION 3.5)
project(IDcontroller LANGUAGES CXX)

find_package(Eigen3 REQUIRED HINTS ${Eigen3_HINT})
find_package(camel-tools-trajectory REQUIRED)
find_package(camel-tools-thread REQUIRED)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${canine-leg-left_util_SOURCE_DIR}/include
)

add_library(${PROJECT_NAME} STATIC
        src/IDController.cpp
        )

add_library(canine-leg-left_controller::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}
        camel-tools-trajectory
        raisim::raisim
        canine-leg-left_util
        camel-tools-thread
        )