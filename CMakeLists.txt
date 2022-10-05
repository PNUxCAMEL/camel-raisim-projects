CMAKE_MINIMUM_REQUIRED(VERSION 3.10)
PROJECT(camel-raisim-projects)

ADD_COMPILE_DEFINITIONS (URDF_RSC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/camel-urdf/")

add_subdirectory(camel-canine-leg-left)
#add_subdirectory(camel-thirdparty/qpOASES)
add_subdirectory(camel-canine)