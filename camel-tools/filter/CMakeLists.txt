cmake_minimum_required(VERSION 3.1.0)

project(filter LANGUAGES CXX)


set(filter_src
        src/LPF.cpp
)

set(filter_hdr
        include/filter.hpp
        include/LPF.hpp
)

add_library(camel-tools-filter SHARED ${filter_src})

# for install headers
install(FILES ${filter_hdr}
        DESTINATION include/camel-tools/
        )

# for create config.cmake file
install(TARGETS camel-tools-filter
        EXPORT camel-tools-filter-export
        LIBRARY DESTINATION lib
        INCLUDES DESTINATION include
        )

install(EXPORT camel-tools-filter-export
        FILE camel-tools-filter-config.cmake
        DESTINATION share/camel-tools-filter/
        EXPORT_LINK_INTERFACE_LIBRARIES
        )