cmake_minimum_required(VERSION 3.5)
project(canine_simulation LANGUAGES CXX)

find_package(camel-tools-thread REQUIRED)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${qpOASES_SOURCE_DIR}/include

        ${canine_gui_SOURCE_DIR}/include

        ${canine_util_SOURCE_DIR}/include

        ${ControlUtils_SOURCE_DIR}/include
        ${PDcontroller_SOURCE_DIR}/include
        ${convexMPC_SOURCE_DIR}/include
)
link_directories(
        ${qpOASES_SOURCE_DIR}/bin
)

add_library(${PROJECT_NAME}
        STATIC
        src/SimulMain.cpp
        src/SimulStateEstimator.cpp
        src/SimulContolPanel.cpp
        src/SimulVisualizer.cpp
        src/SimulCommand.cpp)

target_link_libraries(${PROJECT_NAME}
        PUBLIC
        raisim::raisim
        camel-tools-thread
        PRIVATE
        canine_visualizer::canine_gui
        canine_controller::ControlUtils
        canine_controller::convexMPC
        canine_controller::PDcontroller
)