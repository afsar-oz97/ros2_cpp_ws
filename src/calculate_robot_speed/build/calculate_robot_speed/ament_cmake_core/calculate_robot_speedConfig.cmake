# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_calculate_robot_speed_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED calculate_robot_speed_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(calculate_robot_speed_FOUND FALSE)
  elseif(NOT calculate_robot_speed_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(calculate_robot_speed_FOUND FALSE)
  endif()
  return()
endif()
set(_calculate_robot_speed_CONFIG_INCLUDED TRUE)

# output package information
if(NOT calculate_robot_speed_FIND_QUIETLY)
  message(STATUS "Found calculate_robot_speed: 0.0.0 (${calculate_robot_speed_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'calculate_robot_speed' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${calculate_robot_speed_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(calculate_robot_speed_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${calculate_robot_speed_DIR}/${_extra}")
endforeach()
