cmake_minimum_required(VERSION 3.5)
project(camel-canineV2 LANGUAGES CXX)

find_package(Qt5 CONFIG REQUIRED COMPONENTS Widgets)
find_package(Eigen3 REQUIRED HINTS ${Eigen3_HINT})
find_package(raisim CONFIG REQUIRED)
find_package(camel-tools-thread REQUIRED)
find_package(camel-tools-sensor REQUIRED)

add_subdirectory(canineV2_controller/convexMPC)
add_subdirectory(canineV2_controller/PDcontroller)
add_subdirectory(canineV2_controller/WBController)
add_subdirectory(canineV2_controller/ControlUtils)
add_subdirectory(canineV2_controller/ControlMain)

add_subdirectory(canineV2_visualizer/canineV2_gui)
add_subdirectory(canineV2_visualizer/canineV2_raisim)

add_subdirectory(canineV2_util)
add_subdirectory(canineV2_fsm)

add_subdirectory(canineV2_simulation)

include_directories(
        ${EIGEN3_INCLUDE_DIRS}
        ${qpOASES_SOURCE_DIR}/include

        ${convexMPC_SOURCE_DIR}/include
        ${PDcontroller_SOURCE_DIR}/include
        ${WBController_SOURCE_DIR}/include
        ${ControlUtils_SOURCE_DIR}/include
        ${ControlMain_SOURCE_DIR}/include
        ${PDQPcontrol_SOURCE_DIR}/include

        ${canineV2_gui_SOURCE_DIR}/include
        ${canineV2_raisim_SOURCE_DIR}/include

        ${canineV2_util_SOURCE_DIR}/include
        ${canineV2_fsm_SOURCE_DIR}/include
        ${canineV2_simulation_SOURCE_DIR}/include
)
link_directories(
        ${qpOASES_SOURCE_DIR}/bin
)

include_directories(/usr/share/c++-mscl/source /usr/share/c++-mscl/Boost/include)
link_directories(/usr/share/c++-mscl)
link_libraries(mscl stdc++ pthread)

add_executable(camel-canineV2 canineV2_demo/CanineV2Main.cpp)
target_link_libraries(camel-canineV2
        PUBLIC
        Qt5::Widgets
        PRIVATE
        canineV2_fsm
        canineV2_util
        mscl
)

add_executable(camel-canineV2-simul canineV2_demo/CanineV2Simul.cpp)
target_link_libraries(camel-canineV2-simul
        PUBLIC
        Qt5::Widgets
        PRIVATE
        canineV2_simulation
        canineV2_util
        mscl
)