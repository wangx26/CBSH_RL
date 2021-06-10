cmake_minimum_required(VERSION 3.5)
project(MAPF)

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
set(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/local/share")

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

set(CMAKE_MODULE_PATH
        APPEND "${PROJECT_SOURCE_DIR}/src/log/"
        )

find_package(Boost REQUIRED COMPONENTS
        program_options
        )

find_package(LOG4CPP REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(DPKG_DEV REQUIRED libdpkg)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)

include_directories(${LOG4CPP_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/main.cpp ${SRCS} )

target_link_libraries(${PROJECT_NAME}
        ${Boost_LIBRARIES}
        ${LOG4CPP_LIBRARIES}
        ${OpenCV_LIBS}
        jsoncpp
        )