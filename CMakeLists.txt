cmake_minimum_required(VERSION 3.5.1)
project(state_estimator)
set(CMAKE_BUILD_TYPE Debug) # Release, RelWithDebInfo
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread -pg -O2")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}) 
find_package(catkin REQUIRED cmake_modules)
find_package(Eigen REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  DEPENDS Eigen
  CATKIN_DEPENDS roscpp
  INCLUDE_DIRS include
)

set(SRC ${SRC} 
    src/state_estimator.cpp

    src/signal_handler_linux.cpp
    src/timer.cpp

    src/error_state_kalman_filter.cpp
    src/geometry_library.cpp
)

set(includeDirs ${includeDirs} ${CMAKE_CURRENT_BINARY_DIR})
set(includeDirs ${includeDirs} include)

include_directories( ${includeDirs} )
include_directories( ${catkin_INCLUDE_DIRS} )
include_directories( ${Eigen_INCLUDE_DIRS} )

# executables
add_executable(state_estimator_node src/main.cpp ${SRC})
target_link_libraries (state_estimator_node ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

add_executable(test_expm src/test_expm.cpp ${SRC})
target_link_libraries (test_expm ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

add_executable(test_quat src/test_quaternion.cpp ${SRC})
target_link_libraries (test_quat ${catkin_LIBRARIES} ${Eigen_LIBRARIES})