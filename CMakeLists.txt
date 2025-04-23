cmake_minimum_required(VERSION 3.10)
project(octreevoxelgrid)

find_package(PCL 1.12 REQUIRED)
find_package(VTK 9.1 REQUIRED)
 


include_directories(${PCL_INCLUDE_DIRS})
include_directories(${VTK_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR}/include)



link_directories(${PCL_LIBRARY_DIRS})
link_directories(${VTK_LIBRARY_DIRS})



add_definitions(${PCL_DEFINITIONS})
add_definitions(${VTK_DEFINITIONS})

add_executable(app 
    main.cpp
    src/ArgumentParser.cpp
    src/PCDLoader.cpp
    src/OctreeProcessor.cpp
    src/VoxelGridProcessor.cpp
    src/PointCloudViewer.cpp
    src/GroundRemovalProcessor.cpp  
    src/EuclideanClusterProcessor.cpp   
    src/DBSCANClusteringProcessor.cpp
    src/OBBFittingProcessor.cpp
    src/BackgroundRemovalProcessor.cpp
    src/OutlierRemovalProcessor.cpp
    src/MeanShiftProcessor.cpp
    src/FeatureExtractionProcessor.cpp
    src/FeatureExporter.cpp
)

target_link_libraries(app ${PCL_LIBRARIES} ${VTK_LIBRARIES})  
vtk_module_autoinit(TARGETS app MODULES VTK::RenderingContextOpenGL2)

message("Project: ${PROJECT_NAME} compiled with CMake ${CMAKE_VERSION}")


