cmake_minimum_required(VERSION 3.14)

project(
    acceleration
    VERSION 1.0.0
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)

add_library(acceleration INTERFACE)

target_include_directories(
    acceleration
    INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
              $<INSTALL_INTERFACE:include/acceleration-${PROJECT_VERSION}>)

install(
    TARGETS acceleration
    EXPORT acceleration-targets
    DESTINATION lib/acceleration-${PROJECT_VERSION})

install(DIRECTORY include/ DESTINATION include/acceleration-${PROJECT_VERSION})

install(
    EXPORT acceleration-targets
    FILE acceleration-targets.cmake
    NAMESPACE acceleration::
    DESTINATION lib/cmake/acceleration-${PROJECT_VERSION})

include(CMakePackageConfigHelpers)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/acceleration-config.cmake"
    INSTALL_DESTINATION lib/cmake/acceleration-${PROJECT_VERSION}
    NO_SET_AND_CHECK_MACRO NO_CHECK_REQUIRED_COMPONENTS_MACRO)

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/acceleration-config-version.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/acceleration-config.cmake"
              "${CMAKE_CURRENT_BINARY_DIR}/acceleration-config-version.cmake"
        DESTINATION lib/cmake/acceleration-${PROJECT_VERSION})

export(
    EXPORT acceleration-targets
    FILE "${CMAKE_CURRENT_BINARY_DIR}/acceleration-targets.cmake"
    NAMESPACE acceleration::)
