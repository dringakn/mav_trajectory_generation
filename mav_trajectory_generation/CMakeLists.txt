cmake_minimum_required(VERSION 3.15)
project(mav_trajectory_generation)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(CMAKE_MACOSX_RPATH 0)
add_definitions(-std=c++17)

# Link against system catkin yaml-cpp if installed.
find_package(PkgConfig)
find_package(yaml_cpp_catkin QUIET)
if(${yaml_cpp_catkin_FOUND})
    message(STATUS "Found yaml_cpp_catkin, using instead of system library.")
    set(YamlCpp_LIBRARIES ${yaml_cpp_catkin_LIBRARIES})
    set(YamlCpp_INCLUDE_DIRS ${yaml_cpp_catkin_INCLUDE_DIRS})
else()
    message(STATUS "No yaml_cpp_catkin, using yaml-cpp system library instead.")
    pkg_check_modules(YamlCpp REQUIRED yaml-cpp>=0.5)
endif()

##############################
# Python bindings via Nanobind
##############################
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(Eigen3 REQUIRED)
include_directories(${Python3_INCLUDE_DIRS})
add_subdirectory(externals/nanobind)
nanobind_add_module(mav_trajectory_generation_py src/bindings.cpp)
target_link_libraries(mav_trajectory_generation_py PRIVATE ${PROJECT_NAME} Python3::Python Eigen3::Eigen)
# Build as .so with no “lib” prefix
set_target_properties(mav_trajectory_generation_py PROPERTIES 
  CMAKE_CXX_STANDARD 17
  LINKER_LANGUAGE    CXX
  PREFIX "" 
  SUFFIX ".so"
)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/motion_defines.cpp
  src/polynomial.cpp
  src/segment.cpp
  src/timing.cpp
  src/trajectory.cpp
  src/trajectory_sampling.cpp
  src/vertex.cpp
  src/io.cpp
  src/rpoly/rpoly_ak1.cpp
)
# Link against yaml-cpp.
target_link_libraries(${PROJECT_NAME} ${YamlCpp_LIBRARIES})

############
# BINARIES #
############
cs_add_executable(polynomial_timing_evaluation
  src/polynomial_timing_evaluation.cpp
)
target_link_libraries(polynomial_timing_evaluation ${PROJECT_NAME})

#########
# TESTS #
#########
catkin_add_gtest(test_polynomial
  test/test_polynomial.cpp
)
target_link_libraries(test_polynomial ${PROJECT_NAME} ${catkin_LIBRARIES})

catkin_add_gtest(test_polynomial_optimization
  test/test_polynomial_optimization.cpp
)
target_link_libraries(test_polynomial_optimization ${PROJECT_NAME} ${catkin_LIBRARIES})

##########
# EXPORT #
##########
cs_install()
cs_export()
