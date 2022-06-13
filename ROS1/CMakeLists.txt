################################################################################
#                                                                              #
# Copyright (C) 2022 Fondazione Istitito Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################

# @author Luca Tricerri <luca.tricerri@iit.it>  

cmake_minimum_required(VERSION 3.5)
include(FetchContent)
include(GNUInstallDirs)
#Module creation

project(blocktestros1plugins
        LANGUAGES C CXX
        VERSION 1.1.2)

option(ENABLE_MSVC_WARNINGS "Enable the compilation warnings in windows" ON)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
  if (ENABLE_MSVC_WARNINGS)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
  endif()
  add_compile_options(-bigobj)
else ()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
  set(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -rdynamic")
endif ()

# To build shared libraries in Windows, we set CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS to TRUE.
# See https://cmake.org/cmake/help/v3.4/variable/CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS.html
# See https://blog.kitware.com/create-dlls-on-windows-without-declspec-using-new-cmake-export-all-feature/
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

#Thrift for clock
set(YARP_MINIMUM_REQUIRED_VERSION 3.0.102)

find_package(blocktestcore 2.3.0 REQUIRED)
find_package(YCM REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation)

# Enable RPATH support for installed binaries and libraries
include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_FULL_BINDIR}"
                          LIB_DIRS "${CMAKE_INSTALL_FULL_LIBDIR}"
                          INSTALL_NAME_DIR "${CMAKE_INSTALL_FULL_LIBDIR}"
                          USE_LINK_PATH)


set(SOURCE
        ${SOURCE}
        ${CMAKE_CURRENT_SOURCE_DIR}/src/rosActionDepotStart.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/actionTopicWrite.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/actionTopicRead.cpp
)
set(HEADERS
        ${HEADERS}
        ${CMAKE_CURRENT_SOURCE_DIR}/src/rosActionDepotStart.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/actionTopicWrite.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/actionTopicRead.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/syntax.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/json.hpp
)

add_library(${PROJECT_NAME} MODULE ${SOURCE} ${HEADER})
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src /opt/ros/noetic/include)
target_link_libraries(${PROJECT_NAME} blocktestcore::blocktestcore
                                      #nlohmann_json::nlohmann_json
                                      roscpp
                                      )

#xml template folder
if(WIN32)
    add_custom_command(TARGET ${PROJECT_NAME}
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_CURRENT_LIST_DIR}/xmltemplate
    ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}/$<CONFIG>/xmltemplate)
    else()
    file(COPY ${CMAKE_CURRENT_LIST_DIR}/xmltemplate
            DESTINATION ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})
endif()

install(DIRECTORY tests DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY xmltemplate DESTINATION ${CMAKE_INSTALL_BINDIR})

# Add install target
set_target_properties(${PROJECT_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}/")
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CMAKE_INSTALL_BINDIR}/${PROJECT_NAME})

# Add uninstall target
include(AddUninstallTarget)
