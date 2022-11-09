cmake_minimum_required(VERSION 3.5)
project(PDQPcontrol LANGUAGES CXX)

find_package(camel-tools-trajectory REQUIRED)

find_package(camel-tools-thread REQUIRED)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${qpOASES_SOURCE_DIR}/include
        ${canine_util_SOURCE_DIR}/include
        ${ControlUtils_SOURCE_DIR}/include
        ${WBController_SOURCE_DIR}/include
)

link_directories(
        ${qpOASES_SOURCE_DIR}/bin
)

add_library(${PROJECT_NAME} STATIC
        src/PDQPController.cpp
        src/QPsolver.cpp
        )

add_library(canine_controller::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}
        qpOASES
        camel-tools-trajectory
        raisim::raisim
        canine_util
        canine_controller::ControlUtils
        canine_controller::WBController
        camel-tools-thread
)