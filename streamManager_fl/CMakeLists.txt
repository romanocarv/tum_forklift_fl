cmake_minimum_required(VERSION 3.5)
project(streamManager_fl)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig) #finding pkg-config is a helper tool
find_package(yaml-cpp REQUIRED)

pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)

#including GStreamer header files directory
include_directories(
        ${GLIB_INCLUDE_DIRS}
        ${GSTREAMER_INCLUDE_DIRS}
)

#linking GStreamer library directory
link_directories(
        ${GLIB_LIBRARY_DIRS}
        ${GSTREAMER_LIBRARY_DIRS}
)

add_executable(streamManager_fl src/streamManager_fl.cpp src/gst_pipeline.cpp)
ament_target_dependencies(streamManager_fl rclcpp std_msgs)


#add_executable(client_nano src/client_nano.cpp)
#ament_target_dependencies(client_nano rclcpp std_msgs)

#add_executable(server_nano src/server_nano.cpp)
#ament_target_dependencies(server_nano rclcpp std_msgs)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#linking Gstreamer library with target executable
target_link_libraries(${PROJECT_NAME} ${GSTREAMER_LIBRARIES})
#target_link_libraries(${PROJECT_NAME} gst_pipeline})
target_link_libraries(${PROJECT_NAME} ${YAML_CPP_LIBRARIES})




install(TARGETS
  streamManager_fl
  #listener
  #client_nano
  #server_nano
  DESTINATION lib/${PROJECT_NAME})

ament_package()
