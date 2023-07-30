cmake_minimum_required(VERSION 3.5)
project(sdpo_serial_port)



# Project version
set(${PROJECT_NAME}_MAJOR_VERSION 0)
set(${PROJECT_NAME}_MINOR_VERSION 0)
set(${PROJECT_NAME}_PATCH_VERSION 0)

set(${PROJECT_NAME}_VERSION
  ${${PROJECT_NAME}_MAJOR_VERSION}.${${PROJECT_NAME}_MINOR_VERSION}.${${PROJECT_NAME}_PATCH_VERSION}
)



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



###################################################
#####  >>> ROBOT OPERATING SYSTEM (ROS) 2 <<< #####
###################################################

if("$ENV{ROS_VERSION}" STREQUAL "2")

  # Dependencies
  find_package(Boost REQUIRED COMPONENTS
    date_time
    system
  )



  # Library
  add_library(${PROJECT_NAME} SHARED
    src/AsyncSerial.cpp
  )

  target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    
  )

  target_link_libraries(${PROJECT_NAME} PUBLIC
    Boost::boost
  )

  target_compile_definitions(${PROJECT_NAME} PRIVATE
    "sdpo_serial_port_BUILDING_LIBRARY"
  )



  # Installation rules
  install(
    DIRECTORY include/
    DESTINATION include
  )

  install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )



  # Export targets
  set(export_targets ${export_targets};${PROJECT_NAME})
  export(EXPORT export_${PROJECT_NAME}
    FILE "${PROJECT_BINARY_DIR}/export_${PROJECT_NAME}.cmake"
  )



  # Create Cmake Config files
  set(CONF_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/include")

  configure_file(${PROJECT_NAME}Config.cmake.in
    "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake"
    @ONLY
  )

  configure_file(${PROJECT_NAME}ConfigVersion.cmake.in
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake" @ONLY
  )



  # Installation Cmake Config files rules
  install(
    FILES
      "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake"
      "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    DESTINATION "share/${PROJECT_NAME}/cmake"
    COMPONENT dev
  )

  install(
    EXPORT export_${PROJECT_NAME}
    DESTINATION "share/${PROJECT_NAME}/cmake"
    FILE export_${PROJECT_NAME}.cmake
    COMPONENT dev
  )



  ###################################################
  #####  >>> ROBOT OPERATING SYSTEM (ROS) 1 <<< #####
  ###################################################

elseif("$ENV{ROS_VERSION}" STREQUAL "1")

  message(AUTHOR_WARNING "Project version in ROS 1 may be deprecated")

  # Find catkin macros and libraries
  find_package(catkin REQUIRED COMPONENTS
    roscpp
  )

  find_package(Boost REQUIRED COMPONENTS
    date_time
    system
  )

  # Catkin specific configuration
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp
    DEPENDS Boost
  )

  # Build
  # - additional header files
  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
  )

  # - declare C++ library
  add_library(${PROJECT_NAME}
    src/AsyncSerial.cpp
  )

  # - target dependencies of the library
  add_dependencies(${PROJECT_NAME}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    ${Boost_EXPORTED_TARGETS}
  )

  # - specify libraries to link a library
  target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )

  # - installation rules
  install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
  )



  ##########################
  #####  !!! ERROR !!! #####
  ##########################

else()

  message(FATAL_ERROR "No valid version of ROS is installed or environment variables not set")

endif()
