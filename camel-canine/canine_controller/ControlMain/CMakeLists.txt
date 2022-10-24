cmake_minimum_required(VERSION 3.5)
project(ControlMain LANGUAGES CXX)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${canine_util_SOURCE_DIR}/include
        ${PDcontroller_SOURCE_DIR}/include
        ${convexMPC_SOURCE_DIR}/include
        ${GaitScheduler_SOURCE_DIR}/include
        ${qpOASES_SOURCE_DIR}/include
)
link_directories(
        ${qpOASES_SOURCE_DIR}/bin
)

add_library(${PROJECT_NAME} STATIC
        src/ControllerState.cpp
)

add_library(canine_controller::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}
        canine_controller::convexMPC
        raisim::raisim)