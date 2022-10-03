cmake_minimum_required(VERSION 3.1.0)

project(optimizer LANGUAGES CXX)

find_package(Eigen3 REQUIRED HINTS ${Eigen3_HINT})
include_directories(${EIGEN3_INCLUDE_DIRS})

set(optimizer_src
        src/GradientDescentOptimizer.cpp
)

set(optimizer_hdr
        include/optimizer.hpp
        include/GradientDescentOptimizer.hpp
)

add_library(camel-tools-optimizer SHARED ${optimizer_src})

# for install headers
install(FILES ${optimizer_hdr}
        DESTINATION include/camel-tools/
        )

# for create config.cmake file
install(TARGETS camel-tools-optimizer
        EXPORT camel-tools-optimizer-export
        LIBRARY DESTINATION lib
        INCLUDES DESTINATION include
        )

install(EXPORT camel-tools-optimizer-export
        FILE camel-tools-optimizer-config.cmake
        DESTINATION share/camel-tools-optimizer/
        EXPORT_LINK_INTERFACE_LIBRARIES
        )