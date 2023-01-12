cmake_minimum_required(VERSION 3.5)
project(WBController LANGUAGES CXX)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${qpOASES_SOURCE_DIR}/include
        ${canineV2_util_SOURCE_DIR}/include
        ${ControlUtils_SOURCE_DIR}/include
)

link_directories(
        ${qpOASES_SOURCE_DIR}/bin
)

add_library(${PROJECT_NAME} STATIC
        src/LowPDcontrol.cpp
        )

add_library(canineV2_controller::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}
        qpOASES
        canineV2_controller::ControlUtils
)