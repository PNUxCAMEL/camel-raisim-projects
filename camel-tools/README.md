# CAMEL-TOOLS
>This project is camel tools
- Inho Lee, [inholee8@pusan.ac.kr](inholee8@pusan.ac.kr)
- Hosun Kang, [hosun7379@pusan.ac.kr](hosun7379@pusan.ac.kr)
- Jeahoon Ahn, [dkswogns46@gmail.com](dkswogns46@gmail.com)
---
## How to Install
1. Remove 'example' in the CMakeList.txt
```cmake
CMAKE_MINIMUM_REQUIRED(VERSION 3.1.0)
PROJECT(camel-tools)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_subdirectory(filter)
add_subdirectory(optimizer)
add_subdirectory(sensor)
add_subdirectory(thread)
add_subdirectory(trajectory)
#add_subdirectory(examples)
```
2. Build and install libaries(tools)
```text
cd camel-tools
mkdir build
cd build
cmake ..
make
sudo make install
```
3. (Optional) If you want to use example, add subdirectory in the CMakeList.txt
```cmake
CMAKE_MINIMUM_REQUIRED(VERSION 3.1.0)
PROJECT(camel-tools)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_subdirectory(filter)
add_subdirectory(optimizer)
add_subdirectory(sensor)
add_subdirectory(thread)
add_subdirectory(trajectory)
add_subdirectory(examples)
```