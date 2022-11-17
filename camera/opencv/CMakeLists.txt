# cmake needs this line
cmake_minimum_required(VERSION 3.1)
# Define project name
project(camera_example_project)

if(NOT MSVC)
  include(CMakeFindDependencyMacro)
  find_dependency(Threads)
endif()

include("${CMAKE_CURRENT_LIST_DIR}/@targets_export_name@.cmake")

if(NOT TARGET apriltag AND ${CMAKE_VERSION} VERSION_GREATER "3.10.99")
  # Make imported target globally visible in order to create an ALIAS
  # IMPORTED_GLOBAL is only available in CMake 3.11+
  set_target_properties(apriltag::apriltag PROPERTIES IMPORTED_GLOBAL TRUE)
  # Create alias for backwards compatibility with 3.1.2 and earlier (will be removed in the future - please migrate to apriltag::apriltag)
  add_library(apriltag ALIAS apriltag::apriltag)
endif()
# Find OpenCV, you may need to set OpenCV_DIR variable
# to the absolute path to the directory containing OpenCVConfig.cmake file
# via the command line or GUI
find_package(OpenCV REQUIRED)

# If the package has been found, several variables will
# be set, you can find the full list with descriptions
# in the OpenCVConfig.cmake file.
# Print some message showing some of them
message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

# Declare the executable target built from your sources
add_executable(opencv_demo opencv_demo.cc)

# Link your application with OpenCV libraries
target_link_libraries(opencv_demo PRIVATE ${OpenCV_LIBS})
