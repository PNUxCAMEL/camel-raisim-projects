cmake_minimum_required(VERSION 3.5)
project(ControlMain LANGUAGES CXX)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${canineV2_util_SOURCE_DIR}/include
        ${PDcontroller_SOURCE_DIR}/include
        ${convexMPC_SOURCE_DIR}/include
        ${ControlUtils_SOURCE_DIR}/include
        ${WBController_SOURCE_DIR}/include
        ${qpOASES_SOURCE_DIR}/include
)
link_directories(
        ${qpOASES_SOURCE_DIR}/bin
)

add_library(${PROJECT_NAME} STATIC
        src/ControllerState.cpp
)

add_library(canineV2_controller::${PROJECT_NAME} ALIAS ${PROJECT_NAME})