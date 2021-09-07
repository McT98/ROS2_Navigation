# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)
project(realsense_image_raw)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV 3.2 REQUIRED)
find_package(ament_index_cpp REQUIRED)
#find_package(nav2_costmap_2d REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(realsense2)
find_package(realsense_msgs)


#target_link_libraries(data_pub /home/gewenhao/anaconda3/envs/pytorch/bin/)
#target_link_libraries(data_pub /home/gewenhao/anaconda3/envs/pytorch/lib/)
#link_libraries(/home/gewenhao/anaconda3/envs/pytorch/lib/python3.7/config-3.7m-x86_64-linux-gnu)
#link_libraries(/home/gewenhao/anaconda3/envs/pytorch/lib)
#include_directories(/home/gewenhao/anaconda3/envs/pytorch/include)
#link_libraries(/home/gewenhao/anaconda3/envs/pytorch/lib)
#link_libraries(/home/gewenhao/anaconda3/envs/pytorch/bin)
#set(ENVS_INCLUDE /home/gewenhao/anaconda3/envs/pytorch/include)
#set(ENVS_LIB /home/gewenhao/anaconda3/envs/pytorch/lib)
#set(ENVS_EXECUTABLE /home/gewenhao/anaconda3/envs/pytorch/bin)
#link_libraries("/home/gewenhao/anaconda3/envs/pytorch/lib/libpython3.7m.so")
#find_package(Python3 COMPONENTS Development)
#include_directories(${Python3_INCLUDE_DIRS})
#set(${Python3_INCLUDE_DIRS} /home/gewenhao/anaconda3/envs/pytorch/include/python3.7m)
#set(${Python3_LIBRARIES} /home/gewenhao/anaconda3/envs/pytorch/lib/python3.7)
#set(CMAKE_PREFIX_PATH "/home/gewenhao/anaconda3/envs/pytorch/bin")
#set( Python_ADDITIONAL_VERSIONS 3.7.6 )
#find_package(PythonLibs 3.7 REQUIRED)
#message("hello")
#message("${PYTHON_INCLUDE_DIRS}")
#message("${PYTHON_LIBRARIES}")
#include_directories(${PYTHON_INCLUDE_DIRS})
include_directories(
  ${cv_bridge_INCLUDE_DIRS}
)

add_executable(data_sub ./src/data_sub.cpp)

add_executable(plan_sub ./src/plan_sub.cpp)
#target_link_libraries(data_sub
# ${PYTHON_LIBRARIES}
#)
#target_link_libraries(data_sub
#  ${opencv_libraries}
#)
ament_target_dependencies(data_sub
	"rclcpp"
  #"nav2_costmap_2d"
       	"sensor_msgs"
        "cv_bridge"
        "nav_msgs"
	"ament_index_cpp"
	"class_loader"
)
ament_target_dependencies(plan_sub
  "rclcpp"
  #"nav2_costmap_2d"
        "sensor_msgs"
        "cv_bridge"
        "nav_msgs"
  "ament_index_cpp"
  "class_loader"
)
#set(CMAKE_PREFIX_PATH "/home/gewenhao/anaconda3/envs/pytorch")

#这个到时候可以用
#include_directories(/home/gewenhao/anaconda3/envs/pytorch/include/python3.7m)
#include_directories(/media/gewenhao/disk1/ubuntu_download/SOLO-master/demo)


#add_executable(data_pub ./src/data_pub.cpp /media/gewenhao/disk1/ubuntu_download/SOLO-master/demo/C_Detect.cpp)
add_executable(data_pub ./src/data_pub.cpp)
add_executable(map_process ./src/map_process.cpp)
#add_executable(data_pub ./src/data_pub.cpp)
#target_link_libraries(data_pub
# ${PYTHON_LIBRARIES}
#)

#这个到时候可以用
#target_link_libraries(data_pub /home/gewenhao/anaconda3/envs/pytorch/lib/libpython3.7m.so)
#target_link_libraries(data_pub /home/gewenhao/anaconda3/envs/pytorch/lib)
#target_link_libraries(data_pub /home/gewenhao/anaconda3/envs/pytorch/lib/python3.7/config-3.7m-x86_64-linux-gnu)
#target_link_libraries(data_pub /home/gewenhao/anaconda3/envs/pytorch/lib/python3.7/site-packages)

#link_libraries(/home/gewenhao/anaconda3/envs/pytorch/lib/libpython3.7m.so)
#link_libraries(/home/gewenhao/anaconda3/envs/pytorch/lib)
#link_libraries(/home/gewenhao/anaconda3/envs/pytorch/lib/python3.7/config-3.7m-x86_64-linux-gnu)
#link_libraries(/home/gewenhao/anaconda3/envs/pytorch/lib/python3.7/site-packages)

target_link_libraries(data_pub
  ${opencv_libraries}
)
ament_target_dependencies(data_pub
  #"nav2_costmap_2d"
	"rclcpp"
		"rclcpp_components"
    "map_msgs"
		"nav_msgs"
       	"sensor_msgs"
        "cv_bridge"
	"ament_index_cpp"
	"class_loader"
		"tf2"
		"pcl_conversions"
		"tf2_ros"
		"realsense2"
		"realsense_msgs"
)

target_link_libraries(map_process
  ${opencv_libraries}
)
ament_target_dependencies(map_process
  #"nav2_costmap_2d"
  "rclcpp"
    "rclcpp_components"
    "nav_msgs"
        "sensor_msgs"
        "cv_bridge"
  "ament_index_cpp"
  "class_loader"
    "tf2"
    "pcl_conversions"
    "tf2_ros"
    "realsense2"
    "realsense_msgs"
)

install(TARGETS
  data_sub
  data_pub
  plan_sub
  map_process
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})
ament_package()
