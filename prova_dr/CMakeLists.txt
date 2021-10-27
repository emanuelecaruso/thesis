cmake_minimum_required(VERSION 2.6)

#ds project name
project(dense3Dtracking LANGUAGES C CXX CUDA)

option(USE_CUDA "Use Cuda" ON)

#ds set cmake build flags
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-std=c++11 -O3 -DNDEBUG") #c++11 features, enable maximum optimization and disable debug symbols
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

if (USE_CUDA)
  find_package(CUDA 9.1 REQUIRED)
  include_directories(${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})
  add_definitions(-D USE_CUDA)
endif()

#ds find and set up dependencies
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})




#ds set internal includes (our files)
include_directories(src)
include_directories(src_cuda)
include_directories(libs)

#ds add build targets
add_subdirectory(executables)
