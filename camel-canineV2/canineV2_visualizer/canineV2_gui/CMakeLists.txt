cmake_minimum_required(VERSION 3.5)
project(canineV2_gui LANGUAGES CXX)

set(CMAKE_AUTOMOC ON)
find_package(Qt5 CONFIG REQUIRED COMPONENTS Widgets)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${canineV2_util_SOURCE_DIR}/include
)

add_library(${PROJECT_NAME} STATIC
        include/canineV2_gui/mainwindow.cpp
        include/canineV2_gui/qcustomplot.cpp
)
add_library(canineV2_visualizer::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}
        Qt5::Widgets
        canineV2_util
)

