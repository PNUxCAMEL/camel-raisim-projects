cmake_minimum_required(VERSION 3.5)
project(WBController LANGUAGES CXX)

find_package(RBDL COMPONENTS URDFREADER REQUIRED)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${RBDL_INCLUDE_DIRS}
        ${canine_util_SOURCE_DIR}/include
)


add_library(${PROJECT_NAME}
        src/Controller.cpp
        )

add_library(canine_controller::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}
        canine_util
        ${RBDL_LIBRARY}
        ${RBDL_URDFREADER_LIBRARY}
        )