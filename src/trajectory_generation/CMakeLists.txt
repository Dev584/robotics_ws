cmake_minimum_required(VERSION 3.8)
project(trajectory_generation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

add_library(trajectory_generator_lib src/trajectory_generator.cpp)

target_include_directories(trajectory_generator_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  enable_testing()
  add_executable(test_trajectory_generator test/test_trajectory_generator.cpp)
  target_link_libraries(test_trajectory_generator trajectory_generator_lib GTest::GTest GTest::Main)
  add_test(NAME test_trajectory_generator COMMAND test_trajectory_generator)
endif()

install(TARGETS trajectory_generator_lib LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

ament_package()