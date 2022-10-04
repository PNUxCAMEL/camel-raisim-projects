CMAKE_MINIMUM_REQUIRED(VERSION 3.1.0)
PROJECT(camel-tools)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_subdirectory(filter)
add_subdirectory(optimizer)
add_subdirectory(sensor)
add_subdirectory(thread)
add_subdirectory(trajectory)
#add_subdirectory(examples)
