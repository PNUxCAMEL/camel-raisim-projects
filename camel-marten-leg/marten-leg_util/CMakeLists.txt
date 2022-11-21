cmake_minimum_required(VERSION 3.5)
project(marten-leg_util LANGUAGES CXX)

find_package(camel-tools-thread)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
)

add_library(marten-leg_util
        SHARED
        src/Command.cpp
        src/CanMotor.cpp
        )
target_link_libraries(marten-leg_util
        PUBLIC
        PRIVATE
        pthread
        camel-tools-thread
)