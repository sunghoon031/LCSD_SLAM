cmake_minimum_required(VERSION 2.4.6)
project(dso_ros)

set(BUILD_TYPE Release)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  cv_bridge
  tf
#  message_generation
)

set(DSO_PATH "\home\seonghunlee\ros\dso_private")
message("\n\n---- LOOKING FOR DSO at\n\"${DSO_PATH}\"")


find_package(Pangolin 0.2 REQUIRED)
message("\n\n---- FOUND Pangolin Headers at\n\"${Pangolin_INCLUDE_DIRS}\"")
message("---- FOUND Pangolin Libs at\n\"${Pangolin_LIBRARIES}\"")

	message("--- found PANGOLIN, compiling dso_pangolin library.")
	set(dso_pangolin_SOURCE_FILES 
	  ${DSO_PATH}/src/IOWrapper/Pangolin/KeyFrameDisplay.cpp
	  ${DSO_PATH}/src/IOWrapper/Pangolin/PangolinDSOViewer.cpp)
	set(HAS_PANGOLIN 1)



find_package(OpenCV REQUIRED)
message("\n\n---- FOUND OpenCV Headers at\n\"${OpenCV_INCLUDE_DIRS}\"")
message("---- FOUND OpenCV Libs at\n\"${OpenCV_LIBS}\"")

    message("--- found OpenCV, compiling dso_opencv library.")
    include_directories( ${OpenCV_INCLUDE_DIRS} )
    set(dso_opencv_SOURCE_FILES 
      ${DSO_PATH}/src/IOWrapper/OpenCV/ImageDisplay_OpenCV.cpp
      ${DSO_PATH}/src/IOWrapper/OpenCV/ImageRW_OpenCV.cpp)
    set(HAS_OPENCV 1)

# find_package(Eigen3 REQUIRED)  ## This line is replaced by the following four lines
find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)
set(EIGEN3_INCLUDE_DIR "$ENV{EIGEN3_INCLUDE_DIR}" )
message("\n\n---- FOUND Eigen3 Headers at\n\"${EIGEN3_INCLUDE_DIR}\"")

find_package(LibZip REQUIRED)
# decide if we have ziplib.
if (LIBZIP_LIBRARY)
	message("--- found ziplib (${LIBZIP_VERSION}), compiling with zip capability.")
	add_definitions(-DHAS_ZIPLIB=1)
	include_directories( ${LIBZIP_INCLUDE_DIR_ZIP} ${LIBZIP_INCLUDE_DIR_ZIPCONF} ) 
else()
	message("--- not found ziplib (${LIBZIP_LIBRARY}), compiling without zip capability.")
	set(LIBZIP_LIBRARY "")
endif()

find_package(Boost COMPONENTS system thread)
find_library(DSO_LIBRARY dso ${DSO_PATH}/build/lib)


#add_message_files(
#  DIRECTORY
#  msg
#  FILES
#  marginalized_msg.msg
#)

# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   sensor_msgs
#)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    roscpp
    sensor_msgs
    cv_bridge
#    message_runtime
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

###########
## Build ##
###########

set(CMAKE_CXX_FLAGS
   "${SSE_FLAGS} -O3 -g -std=c++0x -march=native"
#   "${SSE_FLAGS} -O2 -g -std=c++0x -fno-omit-frame-pointer"
)

# Sources files
set(dso_SOURCE_FILES
  ${DSO_PATH}/src/FullSystem/FullSystem.cpp
  ${DSO_PATH}/src/FullSystem/FullSystemOptimize.cpp
  ${DSO_PATH}/src/FullSystem/FullSystemOptPoint.cpp
  ${DSO_PATH}/src/FullSystem/FullSystemDebugStuff.cpp
  ${DSO_PATH}/src/FullSystem/FullSystemMarginalize.cpp
  ${DSO_PATH}/src/FullSystem/Residuals.cpp
  ${DSO_PATH}/src/FullSystem/CoarseTracker.cpp
  ${DSO_PATH}/src/FullSystem/CoarseInitializer.cpp
  ${DSO_PATH}/src/FullSystem/ImmaturePoint.cpp
  ${DSO_PATH}/src/FullSystem/HessianBlocks.cpp
  ${DSO_PATH}/src/FullSystem/PixelSelector2.cpp
  ${DSO_PATH}/src/OptimizationBackend/EnergyFunctional.cpp
  ${DSO_PATH}/src/OptimizationBackend/AccumulatedTopHessian.cpp
  ${DSO_PATH}/src/OptimizationBackend/AccumulatedSCHessian.cpp
  ${DSO_PATH}/src/OptimizationBackend/EnergyFunctionalStructs.cpp
  ${DSO_PATH}/src/util/settings.cpp
  ${DSO_PATH}/src/util/Undistort.cpp
  ${DSO_PATH}/src/util/globalCalib.cpp
)

include_directories(
  include
  ${PROJECT_SOURCE_DIR}/src
  ${DSO_PATH}/src
  ${DSO_PATH}/thirdparty/Sophus
  ${OpenCV_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)


include_directories( ${CSPARSE_INCLUDE_DIR} ${CHOLMOD_INCLUDE_DIR}) 
add_library(dso ${dso_SOURCE_FILES} ${dso_opencv_SOURCE_FILES} ${dso_pangolin_SOURCE_FILES})



## Declare a C++ executable
##add_executable(dso_live src/main.cpp)
add_executable(dso_node src/dso_node.cpp src/ros_output_wrapper.cpp)

#target_link_libraries(dso_live dso boost_system boost_thread cxsparse
target_link_libraries(dso_node dso boost_system boost_thread cxsparse
    ${DSO_LIBRARY}
    ${LIBZIP_LIBRARY}
    ${Pangolin_LIBRARIES}
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES})
