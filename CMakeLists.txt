cmake_minimum_required(VERSION 2.8.4)

project(map-builder)

#tinyXML2
add_library(tinyxml2 STATIC ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/tinyXML/tinyxml2.h ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/tinyXML/tinyxml2.cpp)

#Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

#PCL
find_package(PCL REQUIRED common io visualization)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#octomap
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})

#VTK
find_package (VTK REQUIRED)
include_directories(${VTK_INCLUDE_DIRS})
set  (VTK_LIBRARIES   vtkRendering vtkGraphics vtkHybrid QVTK)

#BOOST
find_package(Boost REQUIRED COMPONENTS system)

add_definitions(-DEIGEN_DONT_VECTORIZE -DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT)

set(SOURCE_FILES main.cpp)

add_executable(map-builder ${SOURCE_FILES})
target_link_libraries(map-builder tinyxml2 ${EIGEN3_INCLUDE_DIR} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
