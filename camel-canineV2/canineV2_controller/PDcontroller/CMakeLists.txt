cmake_minimum_required(VERSION 3.5)
project(PDcontroller LANGUAGES CXX)

find_package(camel-tools-trajectory REQUIRED)

find_package(camel-tools-thread REQUIRED)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${qpOASES_SOURCE_DIR}/include
        ${canineV2_util_SOURCE_DIR}/include
        ${ControlUtils_SOURCE_DIR}/include
        ${WBController_SOURCE_DIR}/include
)

link_directories(
        ${qpOASES_SOURCE_DIR}/bin
)

add_library(${PROJECT_NAME} STATIC
        src/JointPDController.cpp
        )

add_library(canineV2_controller::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}
        qpOASES
        camel-tools-trajectory
        raisim::raisim
        canineV2_util
        canineV2_controller::ControlUtils
        canineV2_controller::WBController
        camel-tools-thread
)