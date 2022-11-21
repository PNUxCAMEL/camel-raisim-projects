cmake_minimum_required(VERSION 3.5)
project(marten-leg_fsm LANGUAGES CXX)

find_package(camel-tools-thread REQUIRED)
find_package(camel-tools-sensor REQUIRED)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${ControlMain_SOURCE_DIR}/include
        ${marten-leg_gui_SOURCE_DIR}/include
        ${marten-leg_raisim_SOURCE_DIR}/include
        ${marten-leg_util_SOURCE_DIR}/include
        ${ControlUtils_SOURCE_DIR}/include
        ${PDcontroller_SOURCE_DIR}/include
        ${IDcontroller_SOURCE_DIR}/include
        ${MPCcontroller_SOURCE_DIR}/include
)

add_library(marten-leg_fsm
        STATIC
        src/MainFSM.cpp
        )

SET(PUBLIC_LIBS
        raisim::raisim
        )

SET(CAMEL_LIBS
        camel-tools-thread
        camel-tools-sensor
        )
SET(MARTEN-LEG_LIBS
        marten-leg_controller::ControlMain
        marten-leg_controller::PDcontroller
        marten-leg_controller::IDcontroller
        marten-leg_controller::MPCcontroller
        marten-leg_controller::ControlUtils
        marten-leg_visualizer::marten-leg_gui
        marten-leg_visualizer::marten-leg_raisim
        marten-leg_util
        )

target_link_libraries(marten-leg_fsm
        PUBLIC
        ${PUBLIC_LIBS}
        PRIVATE
        ${CAMEL_LIBS}
        ${MARTEN-LEG_LIBS}
        )