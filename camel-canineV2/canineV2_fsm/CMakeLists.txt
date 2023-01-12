cmake_minimum_required(VERSION 3.5)
project(canineV2_fsm LANGUAGES CXX)

find_package(camel-tools-thread REQUIRED)
find_package(camel-tools-sensor REQUIRED)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${qpOASES_SOURCE_DIR}/include

        ${ControlMain_SOURCE_DIR}/include
        ${WBController_SOURCE_DIR}/include
        ${ControlUtils_SOURCE_DIR}/include
        ${convexMPC_SOURCE_DIR}/include
        ${PDcontroller_SOURCE_DIR}/include

        ${canineV2_gui_SOURCE_DIR}/include
        ${canineV2_raisim_SOURCE_DIR}/include
        ${GaitScheduler_SOUCRE_DIR}/include
        ${canineV2_util_SOURCE_DIR}/include
)
link_directories(
        ${qpOASES_SOURCE_DIR}/bin
)

include_directories(/usr/share/c++-mscl/source /usr/share/c++-mscl/Boost/include)
link_directories(/usr/share/c++-mscl)
link_libraries(mscl stdc++ pthread)

add_library(canineV2_fsm
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
        canineV2_controller::ControlMain
        canineV2_controller::ControlUtils
        canineV2_controller::PDcontroller
        canineV2_controller::WBController
        canineV2_controller::convexMPC

        canineV2_visualizer::canineV2_gui
        canineV2_visualizer::canineV2_raisim
        canineV2_util
)

target_link_libraries(canineV2_fsm
        PUBLIC
        ${PUBLIC_LIBS}
        ${CAMEL_LIBS}
        ${CANINE_LIBS}
)