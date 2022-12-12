cmake_minimum_required(VERSION 3.5)
project(WBController LANGUAGES CXX)


include_directories(
        include
)


add_executable(${PROJECT_NAME}
        src/Controller.cpp
        )

target_link_libraries(${PROJECT_NAME}
        )