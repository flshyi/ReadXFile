cmake_minimum_required(VERSION 2.8)
project(readxfile)

set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake")

find_package(Boost COMPONENTS thread system filesystem REQUIRED)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -msse2 -msse4.1 -Wno-unused-result")

find_package(OpenCV 3.2 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.13 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.13 not found.")
   endif()
endif()

LIST(APPEND LINK_LIBS 
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
)

include_directories(
  ${Eigen_INCLUDE_DIRS}
  "/usr/include/eigen3"
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}

)
LIST(APPEND SOURCEFILES tinyxml2.cpp read_device_calibration_xml.cpp)

# Create svo library
ADD_LIBRARY(tiny SHARED ${SOURCEFILES})
TARGET_LINK_LIBRARIES(tiny  ${LINK_LIBS})



add_executable(readfile main.cpp)
target_link_libraries( readfile tiny)


