cmake_minimum_required(VERSION 3.11) # FetchContent requires CMake 3.11+
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
# Set default build type to release
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose Release or Debug" FORCE)
endif()

# Name of Project
project(GBPPlanner)
add_compile_definitions(EIGEN_DONT_PARALLELIZE)
find_package(OpenMP REQUIRED)

# Get raylib based on the official raylib CMake project example
# https://github.com/raysan5/raylib/blob/master/projects/CMake/CMakeLists.txt
set(RAYLIB_VERSION 5.5)
find_package(raylib ${RAYLIB_VERSION} QUIET) # QUIET or REQUIRED
if (NOT raylib_FOUND) # If there's none, fetch and build raylib
  include(FetchContent)
  FetchContent_Declare(
    raylib
    DOWNLOAD_EXTRACT_TIMESTAMP OFF
    URL https://github.com/raysan5/raylib/archive/refs/tags/${RAYLIB_VERSION}.tar.gz
  )
  FetchContent_GetProperties(raylib)
  if (NOT raylib_POPULATED) # Have we downloaded raylib yet?
    set(FETCHCONTENT_QUIET NO)
    FetchContent_MakeAvailable(raylib)
  endif()
endif()

#################################################################

add_library(gbp STATIC
    src/gbp/Variable.cpp 
    src/gbp/Factorgraph.cpp 
    src/gbp/Factor.cpp 
)
target_compile_definitions(gbp PUBLIC ASSETS_PATH="${CMAKE_CURRENT_SOURCE_DIR}/assets/")
target_include_directories(gbp PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/inc ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty)
target_link_libraries(gbp PUBLIC OpenMP::OpenMP_CXX raylib)

add_executable(gbpplanner 
    src/main.cpp
    src/Globals.cpp
    src/Robot.cpp
    src/Simulator.cpp
    src/Utils.cpp 
    src/Graphics.cpp 
)
target_link_libraries(gbpplanner PUBLIC gbp)

# Checks if OSX and links appropriate frameworks (only required on MacOS)
if (APPLE)
    target_link_libraries(
      gbpplanner PUBLIC
          "-framework Cocoa, -framework OpenGL, -framework IOKit, -framework CoreVideo"
      )
endif()

#################################################################
