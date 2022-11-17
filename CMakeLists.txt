CMAKE_MINIMUM_REQUIRED(VERSION 3.10)
PROJECT(camel-raisim-projects)

ADD_COMPILE_DEFINITIONS (URDF_RSC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/camel-urdf/")

set(CAMEL_CANINE FALSE)
set(CAMEL_CANINE_LEG_LEFT FALSE)
set(CAMEL_THIRDPARTY_QPOASES FALSE)

if(${CAMEL_CANINE})
    add_subdirectory(camel-canine)
endif()

if(${CAMEL_CANINE_LEG_LEFT})
    add_subdirectory(camel-canine-leg-left)
endif()

if(${CAMEL_THIRDPARTY_QPOASES})
    add_subdirectory(camel-thirdparty/qpOASES)
endif()