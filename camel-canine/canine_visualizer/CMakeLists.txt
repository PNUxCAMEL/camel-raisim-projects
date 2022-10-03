cmake_minimum_required(VERSION 3.5)
project(canine_visualizer LANGUAGES CXX)

set(CMAKE_AUTOMOC ON)
find_package(Qt5 CONFIG REQUIRED COMPONENTS Widgets)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} STATIC
        src/RaisimInit.cpp
        include/canine_gui/mainwindow.cpp
        include/canine_gui/qcustomplot.cpp
)
target_link_libraries(${PROJECT_NAME}
        raisim::raisim
        Qt5::Widgets
)

