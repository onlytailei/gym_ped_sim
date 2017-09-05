cmake_minimum_required(VERSION 2.8.3)
project(actor_plugin)


find_package(roscpp REQUIRED)
find_package(actor_services REQUIRED)
find_package(tf REQUIRED)
find_package(std_msgs REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${actor_services_INCLUDE_DIRS})
include_directories(${tf_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
include_directories(/user/include/sdformat-5.2)
include_directories(./include)
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(actorplugin_ros SHARED src/ActorPlugin_ros.cc)
target_link_libraries(actorplugin_ros ${GAZEBO_LIBRARIES}  ${roscpp_LIBRARIES} ${tf_LIBRARIES})
set_target_properties(actorplugin_ros PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

