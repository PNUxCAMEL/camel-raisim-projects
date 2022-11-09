cmake_minimum_required(VERSION 3.5)
project(canine-leg-left_util LANGUAGES CXX)

find_package(camel-tools-thread)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
)

add_library(canine-leg-left_util
        SHARED
        src/Command.cpp
        )
target_link_libraries(canine-leg-left_util
        PUBLIC
        raisim::raisim
        PRIVATE
        pthread
        camel-tools-thread
)