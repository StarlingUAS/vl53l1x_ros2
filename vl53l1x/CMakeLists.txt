cmake_minimum_required(VERSION 3.5)
project(vl53l1x)

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
find_package(builtin_interfaces REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Build Messages and Services
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MeasurementData.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

# Build vl53l1x
add_subdirectory(lib)

# Build ws281x ros node
add_executable(${PROJECT_NAME}_node src/vl53l1x.cpp)

# Add dependencies
ament_target_dependencies(${PROJECT_NAME}_node rclcpp std_msgs sensor_msgs)

## Link rpi library
target_link_libraries(${PROJECT_NAME}_node rpi_vl53l1x)

# Since we're using SUID bits, we need to leave rpath in for stripped binaries
set_target_properties(${PROJECT_NAME}_node PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

# Ensure cmake can find the msg/srv built as part of this project
rosidl_target_interfaces(${PROJECT_NAME}_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

#############
## Install ##
#############

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
