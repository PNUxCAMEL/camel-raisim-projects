cmake_minimum_required(VERSION 3.5)
project(canineV2_simulation LANGUAGES CXX)

find_package(camel-tools-thread REQUIRED)
find_package(camel-tools-filter REQUIRED)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${qpOASES_SOURCE_DIR}/include

        ${canineV2_gui_SOURCE_DIR}/include

        ${canineV2_util_SOURCE_DIR}/include

        ${ControlUtils_SOURCE_DIR}/include
        ${PDcontroller_SOURCE_DIR}/include
        ${convexMPC_SOURCE_DIR}/include
        ${WBController_SOURCE_DIR}/include
        /usr/share/c++-mscl/source /usr/share/c++-mscl/Boost/include
)
link_directories(
        ${qpOASES_SOURCE_DIR}/bin
        /usr/share/c++-mscl
)

link_libraries(mscl stdc++ pthread)

add_library(${PROJECT_NAME}
        STATIC
        src/SimulMain.cpp
        src/SimulStateEstimator.cpp
        src/SimulContolPanel.cpp
        src/SimulVisualizer.cpp
        src/SimulCommand.cpp
        src/SimulXboxCommand.cpp
        src/SimulKalmanFilter.cpp
)

target_link_libraries(${PROJECT_NAME}
        raisim::raisim
        camel-tools-thread
        camel-tools-filter
        camel-tools-sensor
        canineV2_visualizer::canineV2_gui
        canineV2_controller::ControlUtils
        canineV2_controller::PDcontroller
        canineV2_controller::WBController
        canineV2_controller::convexMPC
        canineV2_util
)