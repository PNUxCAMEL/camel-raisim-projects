cmake_minimum_required(VERSION 3.5)
project(convexMPC LANGUAGES CXX)

find_package(Eigen3 REQUIRED HINTS ${Eigen3_HINT})
find_package(raisim CONFIG REQUIRED)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${qpOASES_SOURCE_DIR}/include
)
link_directories(
        ${qpOASES_SOURCE_DIR}/bin
)

add_library(convexMPC STATIC
        src/Trajectory.cpp
        src/Gait.cpp
        src/qpsolver.cpp
        src/MPCController.cpp
)

add_library(canine_controller::convexMPC ALIAS convexMPC)
target_link_libraries(convexMPC raisim::raisim)