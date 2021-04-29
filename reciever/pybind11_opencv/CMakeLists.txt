project(Test_pybind11)

#**************************************************************************************************
# General cMake settings
#**************************************************************************************************
cmake_minimum_required(VERSION 3.5)

#**************************************************************************************************
# Find Package **************************************************************************************************
find_package(OpenCV REQUIRED)
MESSAGE( " *** OpenCV_INCLUDE_DIRS : " ${OpenCV_INCLUDE_DIRS} )
MESSAGE( " *** OpenCV_LIB_DIRS : " ${OpenCV_LIB_DIRS} )
MESSAGE( " *** OpenCV_LIBS : " ${OpenCV_LIBS} )

find_package(PythonLibs 3 REQUIRED)
MESSAGE( " *** PYTHON_INCLUDE_DIRS : " ${PYTHON_INCLUDE_DIRS} )
MESSAGE( " *** PYTHON_LIBRARIES : " ${PYTHON_LIBRARIES} )

find_package(pybind11 CONFIG REQUIRED)
find_package(Boost REQUIRED)

#**************************************************************************************************
# Include **************************************************************************************************
include_directories(${PYTHON_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS}/opencv4)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${pybind11_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIRS}) 

#**************************************************************************************************
# Set variable **************************************************************************************************
SET(SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/example.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/ndarray_converter.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/PracticalSocket.cpp
)

#**************************************************************************************************
# Set compiler **************************************************************************************************
SET(CMAKE_CXX_FLAGS "-std=c++11 -Wall -O3 -fPIC")

#**************************************************************************************************
# Linker **************************************************************************************************
LINK_DIRECTORIES(
  ${OpenCV_LIB_DIR}
)

#**************************************************************************************************
# Make configuration
#**************************************************************************************************
add_library(example SHARED ${SOURCES})
target_link_libraries(example ${PYTHON_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})
SET_TARGET_PROPERTIES(example PROPERTIES PREFIX "")

install(TARGETS example DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/example)


