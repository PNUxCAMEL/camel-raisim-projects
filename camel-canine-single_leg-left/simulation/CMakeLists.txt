cmake_minimum_required(VERSION 3.5)
project(camel-canine-single_leg-left-simulation LANGUAGES CXX)

#==
# Dependencies
#==
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Qt5 CONFIG REQUIRED COMPONENTS Widgets)
find_package(raisim CONFIG REQUIRED)
find_package(Eigen3 REQUIRED HINTS ${Eigen3_HINT})
find_package(camel-tools-optimizer REQUIRED)
find_package(camel-tools-thread REQUIRED)
find_package(camel-tools-trajectory REQUIRED)

file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/../../../rsc" DESTINATION ${CMAKE_BINARY_DIR}/camel-raisim-projects)

include_directories(
        ${EIGEN3_INCLUDE_DIRS}
        ../../camel-raisim-tools/Simulation/include
        include
)

set(THIRD_PARTY_LIBS
        Qt5::Widgets
        raisim::raisim
        pthread
        rt
#        qpOASES
)

set(CAMEL_LIBS
        camel-tools-optimizer
        camel-tools-trajectory
        camel-tools-thread
)

set(GUI_FILES
        GUI/simulationMainwindow.cpp
        GUI/qcustomplot.cpp)

set(CANINE_SINGLE_LEG_SRC
        ../../camel-raisim-tools/Simulation/src/Controller.cpp
        ../../camel-raisim-tools/Simulation/src/Robot.cpp
        ../../camel-raisim-tools/Simulation/src/Simulation.cpp
        include/SingleLeggedSharedMemory.hpp
        src/controller/SingleLeggedIDController.cpp
        src/controller/SingleLeggedMPCController.cpp
#        src/controller/SingleLeggedMPCqpoases.cpp
        src/controller/SingleLeggedPDController.cpp
        src/SingleLeggedRobot.cpp
        )

add_executable(camel-canine-single_leg-left-simulation
        SingleLeggedSimulation.cpp
        ${GUI_FILES}
        ${CANINE_SINGLE_LEG_SRC})

target_link_libraries(camel-canine-single_leg-left-simulation ${THIRD_PARTY_LIBS} ${CAMEL_LIBS})
