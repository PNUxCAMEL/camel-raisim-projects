cmake_minimum_required(VERSION 3.5)
project(ControlUtils LANGUAGES CXX)

include_directories(
        include
        ${canine-leg-left_util_SOURCE_DIR}/include
)

add_library(${PROJECT_NAME} STATIC
        src/GRFEstimator.cpp
        )

add_library(canine-leg-left_controller::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}
        tensorflow)