cmake_minimum_required(VERSION 3.5)
project(canine_visualizer LANGUAGES CXX)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt5 CONFIG REQUIRED COMPONENTS Widgets)
find_package(raisim CONFIG REQUIRED)
find_package(Eigen3 REQUIRED HINTS ${Eigen3_HINT})

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
)

add_library(canine_visualizer STATIC
        src/CanineRobot.cpp
        GUI/mainwindow.cpp
        GUI/qcustomplot.cpp
)
target_link_libraries(canine_visualizer
        raisim::raisim
        Qt5::Widgets
)