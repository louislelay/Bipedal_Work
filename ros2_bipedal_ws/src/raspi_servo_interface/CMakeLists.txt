cmake_minimum_required(VERSION 3.8)
project(raspi_servo_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  raspi_servo_interface
  SHARED
  src/servo.cpp
  src/servo_control_lib.cpp
)

target_compile_features(raspi_servo_interface PUBLIC cxx_std_17)
target_include_directories(raspi_servo_interface PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/raspi_servo_interface>
)

target_link_libraries(raspi_servo_interface PUBLIC -lwiringPi)

ament_target_dependencies(
  raspi_servo_interface PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "HARDWARE_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface raspi_servo_interface.xml)


# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY     description/launch      description/config      description/urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY     bringup/launch      bringup/config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS raspi_servo_interface
  EXPORT export_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## EXPORTS
ament_export_targets(export_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()