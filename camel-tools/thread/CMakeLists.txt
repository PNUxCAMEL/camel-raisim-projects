cmake_minimum_required(VERSION 3.1.0)

project(thread LANGUAGES CXX)


set(thread_src
        src/ThreadGenerator.cpp
)

set(thread_hdr
        include/ThreadGenerator.hpp
)

add_library(camel-tools-thread SHARED ${thread_src})

# for install headers
install(FILES ${thread_hdr}
        DESTINATION include/camel-tools/
        )

# for create config.cmake file
install(TARGETS camel-tools-thread
        EXPORT camel-tools-thread-export
        LIBRARY DESTINATION lib
        INCLUDES DESTINATION include
        )

install(EXPORT camel-tools-thread-export
        FILE camel-tools-thread-config.cmake
        DESTINATION share/camel-tools-thread/
        EXPORT_LINK_INTERFACE_LIBRARIES
        )