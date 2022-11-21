cmake_minimum_required(VERSION 3.5)
project(ControlMain LANGUAGES CXX)

find_package(raisim CONFIG REQUIRED)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${marten-leg_util_SOURCE_DIR}/include
        ${PDcontroller_SOURCE_DIR}/include
        ${IDcontroller_SOURCE_DIR}/include
        ${MPCcontroller_SOURCE_DIR}/include
)

add_library(${PROJECT_NAME} STATIC
        src/ControllerState.cpp
)

add_library(marten-leg_controller::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}
        raisim::raisim)