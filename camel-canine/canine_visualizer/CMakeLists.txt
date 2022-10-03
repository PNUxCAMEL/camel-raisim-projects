cmake_minimum_required(VERSION 3.5)
project(canine_visualizer LANGUAGES CXX)

set(CMAKE_AUTOMOC ON)
find_package(Qt5 CONFIG REQUIRED COMPONENTS Widgets)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${canine_util_SOURCE_DIR}/include
)

add_library(${PROJECT_NAME} STATIC
        src/RobotVisualization.cpp
        include/canine_gui/mainwindow.cpp
        include/canine_gui/qcustomplot.cpp
)
target_link_libraries(${PROJECT_NAME}
        raisim::raisim
        Qt5::Widgets
        pthread
        canine_util
)

