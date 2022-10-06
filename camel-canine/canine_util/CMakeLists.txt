cmake_minimum_required(VERSION 3.5)
project(canine_util LANGUAGES CXX)

find_package(camel-tools-thread)

include_directories(
        include
)

add_library(canine_util
        SHARED
        src/CanMotorBackward.cpp
        src/CanMotorForward.cpp
        src/Command.cpp
)
target_link_libraries(canine_util
        PRIVATE
        pthread
        camel-tools-thread
)