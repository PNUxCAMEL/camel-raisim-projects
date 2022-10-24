cmake_minimum_required(VERSION 3.5)
project(camel-canine-simulation LANGUAGES CXX)

#==
# Dependencies
#==
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Qt5 CONFIG REQUIRED COMPONENTS Widgets)
find_package(Eigen3 REQUIRED HINTS ${Eigen3_HINT})
find_package(raisim CONFIG REQUIRED)
find_package(camel-tools-thread REQUIRED)

#file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/../../rsc" DESTINATION ${CMAKE_BINARY_DIR}/camel-raisim-projects)

include_directories(
        include
        ../camel-raisim-tools/Simulation/include
        ${EIGEN3_INCLUDE_DIRS}
)

set(THIRD_PARTY_LIBS
        Qt5::Widgets
        raisim::raisim
        pthread
        rt
        )

set(CAMEL_LIBS
        camel-tools-thread
        )

set(GUI_FILES
        GUI/operationMainwindow.cpp
        GUI/qcustomplot.cpp
        )

set(CAMEL_CANINE_SIM_SRC
        ../camel-raisim-tools/Simulation/src/Controller.cpp
        ../camel-raisim-tools/Simulation/src/Robot.cpp
        ../camel-raisim-tools/Simulation/src/Simulation.cpp
        include/CanineSimSharedMemory.hpp
        include/Simulation/CanineSimSimulation.hpp
        )

add_executable(camel-canine-simulation
        src/CanineSimMain.cpp
        ${GUI_FILES}
        ${CAMEL_CANINE_SIM_SRC})

target_link_libraries(camel-canine-simulation ${THIRD_PARTY_LIBS} ${CAMEL_LIBS})
#target_link_libraries(camel-canine-simulation ${THIRD_PARTY_LIBS} ${CAMEL_LIBS} ${GUI_FILES})