cmake_minimum_required(VERSION 3.5)
project(marten-leg_gui LANGUAGES CXX)

set(CMAKE_AUTOMOC ON)
find_package(Qt5 CONFIG REQUIRED COMPONENTS Widgets)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${marten-leg_util_SOURCE_DIR}/include
)

add_library(${PROJECT_NAME} STATIC
        include/marten-leg_gui/mainwindow.cpp
        include/marten-leg_gui/qcustomplot.cpp
)
add_library(marten-leg_visualizer::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}
        Qt5::Widgets
        marten-leg_util
)

