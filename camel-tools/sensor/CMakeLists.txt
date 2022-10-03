cmake_minimum_required(VERSION 3.1.0)

project(sensor LANGUAGES CXX)


set(sensor_src
        src/LoadCell.cpp
#        src/MPU9250.cpp
#        src/RE22SC.cpp
)

set(sensor_hdr
        include/sensor.hpp
        include/LoadCell.hpp
#        include/MPU9250.h
#        include/RE22SC.h
)

add_library(camel-tools-sensor SHARED ${sensor_src})

# for install headers
install(FILES ${sensor_hdr}
        DESTINATION include/camel-tools/
        )

# for create config.cmake file
install(TARGETS camel-tools-sensor
        EXPORT camel-tools-sensor-export
        LIBRARY DESTINATION lib
        INCLUDES DESTINATION include
        )

install(EXPORT camel-tools-sensor-export
        FILE camel-tools-sensor-config.cmake
        DESTINATION share/camel-tools-sensor/
        EXPORT_LINK_INTERFACE_LIBRARIES
        )