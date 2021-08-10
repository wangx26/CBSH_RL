cmake_minimum_required(VERSION 3.5)
project(cbsrl)

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/local/share")
set(LIBRARY_OUTPUT_PATH ./)

include_directories(
        ./src/common
        ./src/algorithm
        ./src/visualization
        ./src/log
        ./src/config
)

aux_source_directory(${PROJECT_SOURCE_DIR}/src/algorithm SRCS)
aux_source_directory(${PROJECT_SOURCE_DIR}/src/algorithm/CBSH2 SRCS)
aux_source_directory(${PROJECT_SOURCE_DIR}/src/common SRCS)
aux_source_directory(${PROJECT_SOURCE_DIR}/src/common/agent SRCS)
aux_source_directory(${PROJECT_SOURCE_DIR}/src/common/mapf_map SRCS)
aux_source_directory(${PROJECT_SOURCE_DIR}/src/config SRCS)
aux_source_directory(${PROJECT_SOURCE_DIR}/src/visualization SRCS)
aux_source_directory(${PROJECT_SOURCE_DIR}/src/log SRCS)
aux_source_directory(${PROJECT_SOURCE_DIR}/src/main.cpp SRCS)

set(CMAKE_MODULE_PATH
        APPEND "${PROJECT_SOURCE_DIR}/src/log/"
        )

find_package(Boost REQUIRED COMPONENTS
        program_options
        )
find_package(Python COMPONENTS Interpreter Development REQUIRED)
#find_package(LOG4CPP REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
#pkg_check_modules(DPKG_DEV REQUIRED libdpkg)
#pkg_check_modules(JSONCPP REQUIRED jsoncpp)

add_subdirectory(third_party/pybind11)
pybind11_add_module(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/src/main.cpp ${SRCS})

#include_directories(${LOG4CPP_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Python_INCLUDE_DIRS})

#add_library(${PROJECT_NAME} SHARED src/main.cpp ${SRCS})

target_link_libraries(${PROJECT_NAME} PRIVATE
        ${Boost_LIBRARIES}
        #${LOG4CPP_LIBRARIES}
        ${Python_LIBRARIES}
        ${OpenCV_LIBS}
        jsoncpp
        -fPIC
        )