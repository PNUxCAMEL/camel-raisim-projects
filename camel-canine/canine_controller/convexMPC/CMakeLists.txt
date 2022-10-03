cmake_minimum_required(VERSION 3.5)
project(convexMPC LANGUAGES CXX)


include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${qpOASES_SOURCE_DIR}/include
)
link_directories(
        ${qpOASES_SOURCE_DIR}/bin
)

add_library(${PROJECT_NAME} STATIC
        src/Trajectory.cpp
        src/Gait.cpp
        src/qpsolver.cpp
        src/MPCController.cpp
)

add_library(canine_controller::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME} raisim::raisim)