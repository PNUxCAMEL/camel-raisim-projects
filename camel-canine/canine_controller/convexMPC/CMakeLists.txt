cmake_minimum_required(VERSION 3.5)
project(convexMPC LANGUAGES CXX)


include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${GaitScheduler_SOURCE_DIR}/include
        ${qpOASES_SOURCE_DIR}/include
        ${canine_util_SOURCE_DIR}/include
)
link_directories(
        ${qpOASES_SOURCE_DIR}/bin
)

add_library(${PROJECT_NAME} STATIC
        src/Trajectory.cpp
        src/qpsolver.cpp
        src/MPCController.cpp
)

add_library(canine_controller::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}
        PUBLIC
        raisim::raisim
        PRIVATE
        canine_controller::GaitScheduler
)