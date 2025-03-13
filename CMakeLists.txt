cmake_minimum_required(VERSION 3.8)

set(target py_mc_rtc)
project(${target} LANGUAGES CXX)

# mc_rtc
#add_project_dependency(mc_rtc REQUIRED)
find_package(mc_rtc REQUIRED)

find_package(Eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})

### debug print variables
##get_cmake_property(_variableNames CACHE_VARIABLES)
#get_cmake_property(_variableNames VARIABLES)
#message("VARIABLES>>>>")
#list (SORT _variableNames)
#foreach (_variableName ${_variableNames})
#  message("${_variableName}=${${_variableName}}")
#endforeach()
#message("VARIABLES<<<<")

### test
add_executable(mc_test test/test.cpp)
target_link_libraries(mc_test PUBLIC mc_rtc::mc_control)

### python module
add_subdirectory(pybind11)
pybind11_add_module(PyMcRtc src/PyMcRtc.cpp src/mc_rtc_class.cpp)
target_link_libraries(PyMcRtc PUBLIC mc_rtc::mc_control)

# EXAMPLE_VERSION_INFO is defined by setup.py and passed into the C++ code as a
# define (VERSION_INFO) here.
#target_compile_definitions(${target}
#  PRIVATE VERSION_INFO=${EXAMPLE_VERSION_INFO})

option(DISABLE_CATKIN OFF)
if(NOT DISABLE_CATKIN)
  find_package(catkin REQUIRED) ###
  catkin_package()
  ### just work for installing to install space
  install(TARGETS PyMcRtc
    DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  )
endif()
