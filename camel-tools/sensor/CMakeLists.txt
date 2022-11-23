cmake_minimum_required(VERSION 3.1.0)

project(sensor LANGUAGES CXX)

#for mscl
include_directories(/usr/share/c++-mscl/source /usr/share/c++-mscl/Boost/include)
link_directories(/usr/share/c++-mscl)
link_libraries(mscl stdc++ pthread realsense2)

set(sensor_src
        src/LoadCell.cpp
        src/LordImu3DmGx5Ahrs.cpp
        src/T265.cpp
#        src/MPU9250.cpp
#        src/RE22SC.cpp
)

set(sensor_hdr
        include/sensor.hpp
        include/LoadCell.hpp
        include/LordImu3DmGx5Ahrs.hpp
        include/T265.hpp
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