cmake_minimum_required(VERSION 3.16)
project(qkrt_core VERSION 0.3.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(QKRT_BUILD_TESTS "Build unit tests" ON)
option(QKRT_BUILD_BENCH "Build benchmarks and simulator apps" ON)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Threads REQUIRED)

file(GLOB QKRT_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
add_library(qkrt_core ${QKRT_SOURCES})
add_library(qkrt::qkrt_core ALIAS qkrt_core)
target_include_directories(qkrt_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(qkrt_core PUBLIC Eigen3::Eigen Threads::Threads)
target_compile_options(qkrt_core PRIVATE -Wall -Wextra -Wpedantic)
set_target_properties(qkrt_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

if(QKRT_BUILD_TESTS)
  enable_testing()
  file(GLOB QKRT_TESTS CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_*.cpp)
  foreach(test_src ${QKRT_TESTS})
    get_filename_component(test_name ${test_src} NAME_WE)
    add_executable(${test_name} ${test_src})
    target_link_libraries(${test_name} PRIVATE qkrt_core)
    target_include_directories(${test_name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
    add_test(NAME ${test_name} COMMAND ${test_name})
  endforeach()
endif()

if(QKRT_BUILD_BENCH)
  file(GLOB QKRT_BENCHES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/bench/bench_*.cpp)
  foreach(bench_src ${QKRT_BENCHES})
    get_filename_component(bench_name ${bench_src} NAME_WE)
    add_executable(${bench_name} ${bench_src})
    target_link_libraries(${bench_name} PRIVATE qkrt_core)
  endforeach()
  file(GLOB QKRT_APPS CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/apps/*.cpp)
  foreach(app_src ${QKRT_APPS})
    get_filename_component(app_name ${app_src} NAME_WE)
    add_executable(${app_name} ${app_src})
    target_link_libraries(${app_name} PRIVATE qkrt_core)
  endforeach()
endif()

# Install as a CMake package so the ROS 2 (ament) wrapper can find_package(qkrt_core).
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
install(TARGETS qkrt_core EXPORT qkrt_coreTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT qkrt_coreTargets NAMESPACE qkrt:: DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/qkrt_core)
configure_package_config_file(cmake/qkrt_coreConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/qkrt_coreConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/qkrt_core)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/qkrt_coreConfigVersion.cmake
  COMPATIBILITY SameMajorVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/qkrt_coreConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/qkrt_coreConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/qkrt_core)
