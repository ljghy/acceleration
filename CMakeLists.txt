cmake_minimum_required(VERSION 3.14)

project(
    acceleration
    VERSION 0.0.1
    LANGUAGES CXX)

add_library(acceleration INTERFACE)
add_library(acceleration::acceleration ALIAS acceleration)

target_include_directories(
    acceleration
    INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
              $<INSTALL_INTERFACE:include/acceleration-${PROJECT_VERSION}>)

target_compile_features(acceleration INTERFACE cxx_std_20)

option(ACCELERATION_DONT_PARALLELIZE "Disable parallelization" OFF)

set(ACCELERATION_COMPILE_DEFINITIONS "")

if(ACCELERATION_DONT_PARALLELIZE)
    list(APPEND ACCELERATION_COMPILE_DEFINITIONS ACC_DONT_PARALLELIZE)
endif()

if(MSVC)
    list(APPEND ACCELERATION_COMPILE_DEFINITIONS NOMINMAX)
endif()

target_compile_definitions(acceleration
                           INTERFACE ${ACCELERATION_COMPILE_DEFINITIONS})

get_directory_property(HAS_PARENT PARENT_DIRECTORY)
if(HAS_PARENT)
    set(ACCELERATION_IS_TOPLEVEL FALSE)
else()
    set(ACCELERATION_IS_TOPLEVEL TRUE)
endif()

option(ACCELERATION_BUILD_EXAMPLES "Build examples" ${ACCELERATION_IS_TOPLEVEL})
if(ACCELERATION_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

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
