cmake_minimum_required(VERSION 3.5)
project(camel-canine LANGUAGES CXX)

#==
# QT5 Dependencies
#==
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt5 CONFIG COMPONENTS Widgets REQUIRED)

include_directories(
        includes

)

set(THIRD_PARTY_LIBS
        Qt5::Widgets
        qpOASES
)



add_executable(camel-canine src/CanineMain.cpp ${GUI_FILES})
target_link_libraries(camel-canine ${THIRD_PARTY_LIBS})