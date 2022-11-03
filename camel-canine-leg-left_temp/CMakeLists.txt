cmake_minimum_required(VERSION 3.5)
project(camel-canine-leg-left LANGUAGES CXX)

find_package(Qt5 CONFIG REQUIRED COMPONENTS Widgets)
find_package(Eigen3 REQUIRED HINTS ${Eigen3_HINT})
find_package(raisim CONFIG REQUIRED)
find_package(camel-tools-thread REQUIRED)
find_package(camel-tools-sensor REQUIRED)

add_subdirectory(canine-leg-left_controller/ControlMain)
add_subdirectory(canine-leg-left_controller/ControlUtils)
add_subdirectory(canine-leg-left_controller/PDcontroller)
add_subdirectory(canine-leg-left_controller/IDcontroller)

add_subdirectory(canine-leg-left_simulation)
add_subdirectory(canine-leg-left_util)
add_subdirectory(canine-leg-left_visualizer/canine-leg-left_gui)

include_directories(
        ${EIGEN3_INCLUDE_DIRS}
        ${PDcontroller_SOURCE_DIR}/include
        ${IDcontroller_SOURCE_DIR}/include
        ${ControlMain_SOURCE_DIR}/include
        ${ControlUtils_SOURCE_DIR}/include
        ${canine-leg-left_gui_SOURCE_DIR}/include
        ${canine-leg-left_util_SOURCE_DIR}/include
        ${canine-leg-left_simulation_SOURCE_DIR}/include
)

add_executable(camel-canine-leg-left-simul canine-leg-left_demo/CanineLegLeftSimul.cpp)
target_link_libraries(camel-canine-leg-left-simul
        PUBLIC
        Qt5::Widgets
        PRIVATE
        canine-leg-left_simulation
)