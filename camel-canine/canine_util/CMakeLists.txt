cmake_minimum_required(VERSION 3.5)
project(canine_util LANGUAGES CXX)

include_directories(
        include
)

add_library(canine_util
        SHARED
        src/Command.cpp
        src/MotorCAN.cpp
)