# COMPILER SETTINGS (default: Release)
# use "-DCMAKE_BUILD_TYPE=Debug" in cmake for a Debug-build
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)

message(STATUS "${PROJECT_NAME} building as ${CMAKE_BUILD_TYPE}")

# COMPILER FLAGS
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(
        "-Wall" "-Wextra" "-Wshadow" "-Wundef" "-pedantic" "-Wnull-dereference"
        "$<$<CONFIG:Release>:-O3>"
        "$<$<CONFIG:Release>:-funroll-loops>"
        "$<$<CONFIG:Debug>:-Og>"
    )
    add_compile_definitions($<$<CONFIG:Release>:NDEBUG>)
endif()

# enables -fPIC in applicable compilers
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
