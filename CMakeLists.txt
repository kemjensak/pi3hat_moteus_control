cmake_minimum_required(VERSION 3.8)
project(pi3hat_moteus_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  ament_cmake_python
  rclcpp
  rclpy
  moteus_msgs
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_executable(moteus_control
  src/pi3hat/pi3hat.cc
  src/pi3hat_moteus_control.cpp
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
      scripts/udp_connector.py
      DESTINATION lib/${PROJECT_NAME}
)

ament_target_dependencies(moteus_control ${THIS_PACKAGE_INCLUDE_DEPENDS})

target_link_libraries(moteus_control /lib/libbcm_host.so)

target_include_directories(
  moteus_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include
)

add_executable(pi3hat_tool 
  src/pi3hat/pi3hat.cc
  src/pi3hat/pi3hat_tool.cc
)

ament_target_dependencies(pi3hat_tool ${THIS_PACKAGE_INCLUDE_DEPENDS})

target_link_libraries(pi3hat_tool /lib/libbcm_host.so)

target_include_directories(
  pi3hat_tool PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include
)

install(
        TARGETS moteus_control
        DESTINATION lib/${PROJECT_NAME}
)

install(
        TARGETS pi3hat_tool
        DESTINATION lib/${PROJECT_NAME}
)

install(
        DIRECTORY include/
        DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)

ament_package()
