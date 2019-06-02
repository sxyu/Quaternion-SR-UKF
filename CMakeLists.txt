cmake_minimum_required( VERSION 2.8 )

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

project( srukf )
set( INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include" )

set( CMAKE_CXX_STACK_SIZE "10000000" )
set( CMAKE_CXX_STANDARD 11 )
set( CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules" )

if( NOT CMAKE_BUILD_TYPE )
  set( CMAKE_BUILD_TYPE Release )
endif()

set( PROJ_NAME "SR-UKF" )
set( OUTPUT_NAME "ukf-example" )

include( CheckCXXCompilerFlag )
CHECK_CXX_COMPILER_FLAG( "-std=c++11" COMPILER_SUPPORTS_CXX11 )
CHECK_CXX_COMPILER_FLAG( "-std=c++0x" COMPILER_SUPPORTS_CXX0X )

if( COMPILER_SUPPORTS_CXX11 )
    set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11" )
elseif( COMPILER_SUPPORTS_CXX0X )
    set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x" )
else()
    message( STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler." )
endif()

if ( CMAKE_COMPILER_IS_GNUCXX )
    set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated -Wno-deprecated-declarations -Wno-attributes -O3 -g" )
endif ( CMAKE_COMPILER_IS_GNUCXX )

if ( NOT DEFINED EIGEN_INCLUDE_DIRS )
    message( STATUS "You may want to specify EIGEN_INCLUDE_DIRS: cmake .. -DEIGEN_INCLUDE_DIRS='eigen/path'" )
    FIND_PACKAGE( Eigen REQUIRED )
endif ()

include_directories(${EIGEN_INCLUDE_DIRS})

set(
  SOURCES
  example.cpp
)

set(
  HEADERS
  UKF.h
)

add_executable( ${PROJ_NAME} ${SOURCES} ${HEADERS} )
set_target_properties( ${PROJ_NAME} PROPERTIES OUTPUT_NAME ${OUTPUT_NAME} )

if ( MSVC )
    set_property(TARGET ${PROJ_NAME} APPEND PROPERTY LINK_FLAGS /DEBUG)
else ()
    target_link_libraries( ${PROJ_NAME} -pthread )
endif ( MSVC )

# Create source group for headers
source_group( "Header Files" FILES ${HEADERS} )
