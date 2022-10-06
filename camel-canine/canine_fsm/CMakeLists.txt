cmake_minimum_required(VERSION 3.5)
project(canine_fsm LANGUAGES CXX)

find_package(camel-tools-thread REQUIRED)
find_package(camel-tools-sensor REQUIRED)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${ControlMain_SOURCE_DIR}/include
        ${canine_gui_SOURCE_DIR}/include
        ${canine_raisim_SOURCE_DIR}/include
        ${canine_util_SOURCE_DIR}/include
        ${PDcontroller_SOURCE_DIR}/include
)

include_directories(/usr/share/c++-mscl/source /usr/share/c++-mscl/Boost/include)
link_directories(/usr/share/c++-mscl)
link_libraries(mscl stdc++ pthread)

add_library(canine_fsm
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
SET(CANINE_LIBS
        canine_controller::convexMPC
        canine_controller::ControlMain
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