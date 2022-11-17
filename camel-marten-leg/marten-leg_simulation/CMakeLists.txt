cmake_minimum_required(VERSION 3.5)
project(marten-leg_simulation LANGUAGES CXX)

find_package(camel-tools-thread REQUIRED)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${marten-leg_gui_SOURCE_DIR}/include
        ${marten-leg_util_SOURCE_DIR}/include
        ${PDcontroller_SOURCE_DIR}/include
        ${IDcontroller_SOURCE_DIR}/include
        ${MPCcontroller_SOURCE_DIR}/include
        ${GRFcontroller_SOURCE_DIR}/include
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
        marten-leg_visualizer::marten-leg_gui
        marten-leg_controller::PDcontroller
        marten-leg_controller::IDcontroller
        marten-leg_controller::MPCcontroller
        marten-leg_controller::GRFcontroller
        marten-leg_controller::ControlUtils
        pthread
)