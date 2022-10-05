cmake_minimum_required(VERSION 3.1.0)

project(trajectory LANGUAGES CXX)

find_package(Eigen3 REQUIRED HINTS ${Eigen3_HINT})
include_directories(${EIGEN3_INCLUDE_DIRS})

set(trajectory_src
        src/CubicTrajectoryGenerator.cpp
        src/QuinticTrajectoryGenerator.cpp
        src/SineTrajectoryGenerator.cpp
)

set(trajectory_hdr
        include/trajectory.hpp
        include/SineTrajectoryGenerator.hpp
        include/QuinticTrajectoryGenerator.hpp
        include/CubicTrajectoryGenerator.hpp
)

add_library(camel-tools-trajectory SHARED ${trajectory_src})

# for install headers
install(FILES ${trajectory_hdr}
        DESTINATION include/camel-tools/
        )

# for create config.cmake file
install(TARGETS camel-tools-trajectory
        EXPORT camel-tools-trajectory-export
        LIBRARY DESTINATION lib
        INCLUDES DESTINATION include
        )

install(EXPORT camel-tools-trajectory-export
        FILE camel-tools-trajectory-config.cmake
        DESTINATION share/camel-tools-trajectory/
        EXPORT_LINK_INTERFACE_LIBRARIES
        )