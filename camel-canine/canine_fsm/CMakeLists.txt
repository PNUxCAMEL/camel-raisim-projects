cmake_minimum_required(VERSION 3.5)
project(canine_fsm LANGUAGES CXX)

find_package(camel-tools-thread REQUIRED)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${PDcontroller_SOURCE_DIR}/include
        ${canine_gui_SOURCE_DIR}/include
        ${canine_raisim_SOURCE_DIR}/include
        ${canine_util_SOURCE_DIR}/include
)

add_library(canine_fsm
        STATIC
        src/MainFSM.cpp
)

SET(PUBLIC_LIBS
        raisim::raisim
)

SET(CAMEL_LIBS
        camel-tools-thread
)
SET(CANINE_LIBS
        canine_controller::convexMPC
        canine_controller::PDcontroller
        canine_visualizer::canine_gui
        canine_visualizer::canine_raisim
        canine_util
)

target_link_libraries(canine_fsm
        PUBLIC
        ${PUBLIC_LIBS}
        PRIVATE
        ${CAMEL_LIBS}
        ${CANINE_LIBS}
)