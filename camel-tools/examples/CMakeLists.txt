cmake_minimum_required(VERSION 3.1.0)

project(examples LANGUAGES CXX)

find_package(Eigen3 REQUIRED HINTS ${Eigen3_HINT})
find_package(camel-tools-filter REQUIRED)
find_package(camel-tools-optimizer REQUIRED)
find_package(camel-tools-sensor REQUIRED)
find_package(camel-tools-thread REQUIRED)
find_package(camel-tools-trajectory REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(/usr/share/c++-mscl/source /usr/share/c++-mscl/Boost/include)
link_directories(/usr/share/c++-mscl)
link_libraries(mscl stdc++ pthread)

set(CAMEL_LIBS
        camel-tools-filter
        camel-tools-optimizer
        camel-tools-sensor
        camel-tools-thread
        camel-tools-trajectory
        )

#add_executable(example-CSV_file_write src/example_CSVFileWrite.cpp.cpp)
add_executable(example-Eigen src/example_Eigen.cpp)
#add_executable(example-sensor-RE22SC src/example_EncoderRE22SC.cpp.cpp)
add_executable(example-sensor-LoadCell src/example_SensorLoadCell.cpp)
add_executable(example-sensor-LordIMU src/example_IMU-Lord.cpp)
add_executable(example-optimizer-gradient_descent src/example_GradientDescentOptimizer.cpp)
add_executable(example-thread src/example_RealTimeThread.cpp)
#add_executable(example-SGDModel src/example_SGDmodel.cpp)
add_executable(example-trajectory src/example_TrajectoryGenerator.cpp)

#target_link_libraries(example-CSV_file_write ${CAMEL_LIBS})
target_link_libraries(example-Eigen ${CAMEL_LIBS})
#target_link_libraries(example-sensor-RE22SC ${CAMEL_LIBS})
target_link_libraries(example-sensor-LoadCell ${CAMEL_LIBS})
target_link_libraries(example-sensor-LordIMU ${CAMEL_LIBS})
target_link_libraries(example-optimizer-gradient_descent ${CAMEL_LIBS})
target_link_libraries(example-thread ${CAMEL_LIBS} pthread)
#target_link_libraries(example-SGDModel ${CAMEL_LIBS})
target_link_libraries(example-trajectory ${CAMEL_LIBS})
