# Copyright (c) Huawei Technologies Co., Ltd. 2020. All rights reserved.
cmake_minimum_required(VERSION 3.5.1)
project(InferOfflineVideo)

set(PROJECT_SRC_ROOT ${CMAKE_CURRENT_LIST_DIR})
set(CMAKE_MODULE_PATH ${PROJECT_SRC_ROOT}/CMake)
set(CMAKE_SKIP_BUILD_RPATH True)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SRC_ROOT}/dist)
add_compile_options(-std=c++11 -fPIE -fstack-protector-all -Wreturn-type)
add_definitions(-DENABLE_DVPP_INTERFACE)
add_definitions(-DASCEND_MODULE_USE_ACL)

# Check environment variable
if(NOT DEFINED ENV{ASCEND_HOME})
    message(FATAL_ERROR "please define environment variable:ASCEND_HOME")
endif()

# Set include directory and library directory
if (NOT DEFINED ENV{FFMPEG_PATH})
    message(FATAL_ERROR "Please define FFMPEG_PATH first!")
else()
    #message("FFMPEG_PATH $ENV{FFMPEG_PATH}")
    set(FFMPEG_PATH $ENV{FFMPEG_PATH})
endif()

set(CommonFolder ${PROJECT_SRC_ROOT}/Common)
set(AscendBaseFolder ${PROJECT_SRC_ROOT}/../ascendbase)

# Find ffmpeg
find_package(FFMPEG REQUIRED)

# Find acllib
set(ACL_INC_DIR $ENV{ASCEND_HOME}/$ENV{ASCEND_VERSION}/$ENV{ARCH_PATTERN}/include)

# Find ascendbase
set(ASCEND_BASE_DIR ${AscendBaseFolder}/src/Base)
get_filename_component(ASCEND_BASE_ABS_DIR ${ASCEND_BASE_DIR} ABSOLUTE)

file(GLOB_RECURSE ASCEND_BASE_SRC_FILES
    ${ASCEND_BASE_ABS_DIR}/BlockingQueue/*cpp
    ${ASCEND_BASE_ABS_DIR}/CommandParser/*cpp
    ${ASCEND_BASE_ABS_DIR}/ConfigParser/*cpp
    ${ASCEND_BASE_ABS_DIR}/DvppCommon/*cpp
    ${ASCEND_BASE_ABS_DIR}/ErrorCode/*cpp
    ${ASCEND_BASE_ABS_DIR}/FileManager/*cpp
    ${ASCEND_BASE_ABS_DIR}/Framework/ModelProcess/*cpp
    ${ASCEND_BASE_ABS_DIR}/Framework/ModuleManager/*cpp
    ${ASCEND_BASE_ABS_DIR}/Log/*cpp
    ${ASCEND_BASE_ABS_DIR}/PointerDeleter/*cpp
    ${ASCEND_BASE_ABS_DIR}/Statistic/*cpp
    ${ASCEND_BASE_ABS_DIR}/ResourceManager/*cpp
)

# Find Header
set(HEADER_FILE_DIR
    ${ACL_INC_DIR}
    ${ASCEND_BASE_DIR}
    ${ASCEND_BASE_DIR}/Framework
    ${FFMPEG_INCLUDE_DIRS}
    ${PROJECT_SRC_ROOT}/
    ${PROJECT_SRC_ROOT}/Common
    ${PROJECT_SRC_ROOT}/Module
)
include_directories(${HEADER_FILE_DIR})

# Find Src
file(GLOB_RECURSE SRC_FILE
    ${PROJECT_SRC_ROOT}/main.cpp
    ${PROJECT_SRC_ROOT}/Common/*.cpp
    ${PROJECT_SRC_ROOT}/Module/StreamPuller/*.cpp
    ${PROJECT_SRC_ROOT}/Module/VideoDecoder/*.cpp
    ${PROJECT_SRC_ROOT}/Module/ModelInfer/*.cpp
    ${PROJECT_SRC_ROOT}/Module/PostProcess/*.cpp
)

set(SOURCE_FILE
    ${SRC_FILE}
    ${ASCEND_BASE_SRC_FILES}
)

# set the share library directory
set(ACL_LIB_DIR $ENV{ASCEND_HOME}/$ENV{ASCEND_VERSION}/$ENV{ARCH_PATTERN}/lib64/stub)
link_directories(${ACL_LIB_DIR})

# Set the target executable file
add_executable(main ${SOURCE_FILE})

target_link_libraries(main ascendcl acl_dvpp ${FFMPEG_LIBRARIES} pthread -Wl,-z,relro,-z,now,-z,noexecstack -pie -s)
