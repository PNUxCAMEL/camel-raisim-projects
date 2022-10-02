cmake_minimum_required(VERSION 3.11)
project(QcustomPlot)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt5 CONFIG COMPONENTS Widgets REQUIRED)

include_directories(include)

add_library(QcustomPlot src/qcustomplot.cpp)
target_link_libraries(QcustomPlot Qt5::Widgets)
