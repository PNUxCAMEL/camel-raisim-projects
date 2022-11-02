cmake_minimum_required(VERSION 3.5)
project(canine-leg-left_simulation LANGUAGES CXX)

find_package(camel-tools-thread REQUIRED)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${canine-leg-left_gui_SOURCE_DIR}/include
        ${canine-leg-left_util_SOURCE_DIR}/include
        ${PDcontroller_SOURCE_DIR}/include
        ${ControlUtils_SOURCE_DIR}/include
)

add_library(${PROJECT_NAME}
        STATIC
        src/SimulMain.cpp
        src/SimulContolPanel.cpp
        src/SimulVisualizer.cpp
        src/SimulCommand.cpp)

target_link_libraries(${PROJECT_NAME}
        PUBLIC
        raisim::raisim
        camel-tools-thread
        PRIVATE
        canine-leg-left_visualizer::canine-leg-left_gui
        canine-leg-left_controller::PDcontroller
        canine-leg-left_controller::ControlUtils
        pthread
)