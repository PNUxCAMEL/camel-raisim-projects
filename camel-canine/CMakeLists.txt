cmake_minimum_required(VERSION 3.5)
project(camel-canine LANGUAGES CXX)

find_package(Eigen3 REQUIRED HINTS ${Eigen3_HINT})
find_package(raisim CONFIG REQUIRED)
find_package(camel-tools-thread REQUIRED)

add_subdirectory(canine_controller/convexMPC)
add_subdirectory(canine_controller/PDcontroller)
add_subdirectory(canine_visualizer/canine_gui)
add_subdirectory(canine_visualizer/canine_raisim)
add_subdirectory(canine_util)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${convexMPC_SOURCE_DIR}/include
        ${PDcontroller_SOURCE_DIR}/include
        ${canine_gui_SOURCE_DIR}/include
        ${canine_raisim_SOURCE_DIR}/include
        ${canine_util_SOURCE_DIR}/include
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

add_executable(camel-canine src/CanineMain.cpp )
target_link_libraries(camel-canine
        PUBLIC
        ${CAMEL_LIBS}
        PRIVATE
        ${CANINE_LIBS}
)