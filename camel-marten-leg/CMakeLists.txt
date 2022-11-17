cmake_minimum_required(VERSION 3.5)
project(camel-marten-leg LANGUAGES CXX)

find_package(Qt5 CONFIG REQUIRED COMPONENTS Widgets)
find_package(Eigen3 REQUIRED HINTS ${Eigen3_HINT})
find_package(raisim CONFIG REQUIRED)
find_package(camel-tools-thread REQUIRED)
find_package(camel-tools-sensor REQUIRED)

add_subdirectory(marten-leg_controller/ControlMain)
add_subdirectory(marten-leg_controller/ControlUtils)
add_subdirectory(marten-leg_controller/PDcontroller)
add_subdirectory(marten-leg_controller/IDcontroller)
add_subdirectory(marten-leg_controller/MPCcontroller)
add_subdirectory(marten-leg_controller/GRFcontroller)

add_subdirectory(marten-leg_simulation)
add_subdirectory(marten-leg_util)
add_subdirectory(marten-leg_visualizer/marten-leg_gui)

include_directories(
        ${EIGEN3_INCLUDE_DIRS}
        ${PDcontroller_SOURCE_DIR}/include
        ${IDcontroller_SOURCE_DIR}/include
        ${MPCcontroller_SOURCE_DIR}/include
        ${GRFcontroller_SOURCE_DIR}/include
        ${ControlMain_SOURCE_DIR}/include
        ${ControlUtils_SOURCE_DIR}/include
        ${marten-leg_gui_SOURCE_DIR}/include
        ${marten-leg_util_SOURCE_DIR}/include
        ${marten-leg_simulation_SOURCE_DIR}/include
)

add_executable(camel-marten-leg-simul marten-leg_demo/MartenLegSimul.cpp)
target_link_libraries(camel-marten-leg-simul
        PUBLIC
        Qt5::Widgets
        PRIVATE
        marten-leg_simulation
)