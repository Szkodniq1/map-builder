cmake_minimum_required(VERSION 2.8.4)

project(map-builder)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -Wconversion -std=c++11 -pthread")

# Folders
SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)

# Settings
INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}")
INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/resources")
INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/include")
INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/include/Defs")

# Executable output directory
SET(RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/bin${OUTPUT_DIRECTORY_POSTFIX} CACHE PATH "Executable output directory")
mark_as_advanced(RUNTIME_OUTPUT_DIRECTORY)

# Dynamic library output directory
SET(LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/bin${OUTPUT_DIRECTORY_POSTFIX} CACHE PATH "Dynamic library output directory")
mark_as_advanced(LIBRARY_OUTPUT_DIRECTORY)

# Static library output directory
SET(ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/lib${OUTPUT_DIRECTORY_POSTFIX} CACHE PATH "Static library output directory")
mark_as_advanced(ARCHIVE_OUTPUT_DIRECTORY)

SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${RUNTIME_OUTPUT_DIRECTORY})
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${LIBRARY_OUTPUT_DIRECTORY})
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${ARCHIVE_OUTPUT_DIRECTORY})
foreach(CONFIGURATION_TYPE ${CMAKE_CONFIGURATION_TYPES})
        string(TOUPPER ${CONFIGURATION_TYPE} CONFIGURATION_TYPE)
        SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${CONFIGURATION_TYPE} ${RUNTIME_OUTPUT_DIRECTORY})
        SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${CONFIGURATION_TYPE} ${LIBRARY_OUTPUT_DIRECTORY})
        SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${CONFIGURATION_TYPE} ${ARCHIVE_OUTPUT_DIRECTORY})
endforeach(CONFIGURATION_TYPE CMAKE_CONFIGURATION_TYPES)

if(MSVC)
  SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc")
endif()

#tinyXML2
add_library(tinyxml2 STATIC ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/tinyXML/tinyxml2.h ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/tinyXML/tinyxml2.cpp)

#Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

#PCL
#find_package(PCL REQUIRED common io visualization)
#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#octomap
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})

#VTK
#find_package (VTK REQUIRED)
#include_directories(${VTK_INCLUDE_DIRS})
#set  (VTK_LIBRARIES   vtkRendering vtkGraphics vtkHybrid QVTK)

#BOOST
find_package(Boost COMPONENTS system)

    #QGLViewer library
    set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
    FIND_PACKAGE(OpenGL)
    include_directories(${OPENGL_INCLUDE_DIR})
    link_directories(${OPENGL_LIBRARY})

    #QGLViewer library
    FIND_PACKAGE(QGLViewer)
    include_directories(${QGLVIEWER_INCLUDE_DIR})
    include_directories(${QT_INCLUDES})
    link_directories(${QGLVIEWER_LIBRARY})

add_definitions(-DEIGEN_DONT_VECTORIZE -DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT)

###############################################################################
#
#  MAPPING defs library
#
###############################################################################

#if(BUILD_DEFS)
        file(GLOB DEFS_SOURCES
            "${CMAKE_SOURCE_DIR}/src/Defs/*.cpp"
        )
        file(GLOB DEFS_HEADERS
            "${CMAKE_SOURCE_DIR}/include/Defs/*.h"
        )
        ADD_LIBRARY(Defs STATIC ${DEFS_SOURCES} ${DEFS_HEADERS})
        set_target_properties(Defs PROPERTIES LINKER_LANGUAGE CXX)
# 	INSTALL(TARGETS walkersDefs RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${DEFS_HEADERS} DESTINATION include/Defs/)

#endif(BUILD_DEFS)

###############################################################################
#
# MAPPING Utilities library
#
###############################################################################

#if(BUILD_WALKERS_UTILITIES)
        file(GLOB UTILITIES_SOURCES
            "${CMAKE_SOURCE_DIR}/src/Utilities/*.cpp"
        )
        file(GLOB UTILITIES_HEADERS
            "${CMAKE_SOURCE_DIR}/include/Utilities/*.h"
        )
        add_library(Utilities STATIC ${UTILITIES_SOURCES} ${UTILITIES_HEADERS})
#        INSTALL(TARGETS Utilities RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
#        INSTALL(FILES ${TRACKER_HEADERS} DESTINATION include/walkers/Utilities/)
#endif(BUILD_WALKERS_UTILITIES)

###############################################################################
#
# MAPPING Map library
#
###############################################################################

#if(BUILD_WALKERS_UTILITIES)
        file(GLOB MAP_SOURCES
            "${CMAKE_SOURCE_DIR}/src/Map/*.cpp"
        )
        file(GLOB MAP_HEADERS
            "${CMAKE_SOURCE_DIR}/include/Map/*.h"
        )
        add_library(Map STATIC ${MAP_SOURCES} ${MAP_HEADERS})
        TARGET_LINK_LIBRARIES(Map tinyxml2 Utilities)
#        INSTALL(TARGETS Utilities RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
#        INSTALL(FILES ${TRACKER_HEADERS} DESTINATION include/walkers/Utilities/)
#endif(BUILD_WALKERS_UTILITIES)

###############################################################################
#
# MAPPING Visualizer library
#
###############################################################################

#if(BUILD_SIMULATOR AND BUILD_VISUALIZER)
        file(GLOB VISUALIZER_SOURCES
            "${CMAKE_SOURCE_DIR}/src/Visualizer/*.cpp"
        )
        file(GLOB VISUALIZER_HEADERS
            "${CMAKE_SOURCE_DIR}/include/Visualizer/*.h"
        )
        ADD_LIBRARY(Visualizer STATIC ${VISUALIZER_SOURCES} ${VISUALIZER_HEADERS})
        TARGET_LINK_LIBRARIES(Visualizer tinyxml2 ${QGLVIEWER_LIBRARY} ${QT_LIBRARIES} QtCore QtXml QtOpenGL QtGui ${OPENGL_LIBRARY} )
#        INSTALL(TARGETS Utilities RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
#        INSTALL(FILES ${VISUALIZER_HEADERS} DESTINATION include/walkers/Visualizer/)
#endif(BUILD_SIMULATOR AND BUILD_VISUALIZER)

#Qt
#QT5_WRAP_CPP(MOC_SRC_H
#    include/octothread.h
#    include/Defs/defs.h
#    include/Grabber/grabber.h
#    include/Grabber/kinect_grabber.h
#    include/Utilities/observer.h
#    include/Map/map.h
#    include/Map/octomap.h
#    include/Point/point.h
#    include/Point/octopoint.h)

include_directories(include ${CMAKE_CURRENT_BINARY_DIR})
#file(GLOB_RECURSE HEADER_FILES include *.h)
#file(GLOB SOURCES main.cpp "src/*.cpp" "src/Grabber/*.cpp" "src/Utilities/*.cpp" "src/Map/*.cpp" "src/Point/*.cpp")

#add_executable(map-builder ${SOURCES} ${HEADER_FILES} ${MOC_SRC_H})
#target_link_libraries(map-builder tinyxml2 ${EIGEN3_INCLUDE_DIR} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})

###############################################################################
#
# MAPPING demo
#
###############################################################################

#if(BUILD_SIMULATOR AND BUILD_VISUALIZER AND BUILD_SIMULATOR_DEMO AND BUILD_SIMULATOR_DEMO_MAIN)
        SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath ./")
        SET(DEMO_SOURCES ./main.cpp)
        ADD_EXECUTABLE(mapping ${DEMO_SOURCES})
        TARGET_LINK_LIBRARIES(mapping tinyxml2 Defs Map Visualizer ${QGLVIEWER_LIBRARY} ${QT_LIBRARIES} ${QT_LIBRARIES} QtCore QtXml QtOpenGL QtGui ${OPENGL_LIBRARY} glut)
        INSTALL(TARGETS mapping RUNTIME DESTINATION bin)

#endif(BUILD_SIMULATOR AND BUILD_VISUALIZER AND BUILD_SIMULATOR_DEMO AND BUILD_SIMULATOR_DEMO_MAIN)
