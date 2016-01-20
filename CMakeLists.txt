cmake_minimum_required(VERSION 3.3)
project(pcl_cloud_registration)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#set(CMAKE_EXE_LINKER_FLAGS "-Wl,-export-dynamic")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_HOME_DIRECTORY}/bin)

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${CMAKE_HOME_DIRECTORY}/include)

set(SOURCE_FILES
        src/main.cpp
        include/apps/App.hpp
        src/apps/App.cpp
        include/nodes/Node.hpp
        include/nodes/Producer.hpp
        include/Buffer.hpp
        src/nodes/Node.cpp
        include/types/Timing.hpp
        include/nodes/TimedCloudGrabberProducer.hpp
        include/apps/LiveVisualizerApp.hpp
        include/util/Logger.hpp
        src/util/Logger.cpp
        src/apps/LiveVisualizerApp.cpp
        src/util/Timer.cpp
        include/util/Timer.hpp
        include/nodes/TimedImageGrabberProducer.hpp
        include/nodes/TimedDataGrabberProducer.hpp
        include/nodes/MultiSyncedCloudImageGrabberProducer.hpp
        include/types/GuardedGrabber.hpp
        include/nodes/Synchronizer.hpp
        include/types/TimedCloud.hpp
        include/types/TimedImage.hpp
        src/nodes/Synchronizer.cpp
        include/nodes/Processor.hpp
        include/nodes/SyncProcessor.hpp
        include/types/TimedData.hpp
        include/types/TimedCloudAndImagePair.hpp
        include/nodes/MultiCloudAndImagePairViewer.hpp
        include/nodes/Viewer.hpp
        include/nodes/Calibrator.hpp
        src/util/KeypointSorter.cpp
        include/util/KeypointSorter.hpp
        src/util/SquareDetector.cpp
        include/util/SquareDetector.hpp
        src/util/Palette.cpp
        include/util/Palette.hpp
        include/util/CameraId.hpp
        src/util/CameraConfig.cpp
        include/util/CameraConfig.hpp
        src/util/SquareSorter.cpp
        include/util/SquareSorter.hpp include/util/CameraIntrinsicsLoader.hpp src/util/CameraIntrinsicsLoader.cpp include/nodes/CalibratorAcircles.hpp)

add_executable(pcl_cloud_registration ${SOURCE_FILES})

target_link_libraries(pcl_cloud_registration ${PCL_LIBRARIES})
target_link_libraries(pcl_cloud_registration ${OpenCV_LIBS})
