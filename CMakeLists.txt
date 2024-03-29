cmake_minimum_required(VERSION 3.14)
project(viterbi-snakes VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(BUILD_TESTS "Build test code" ON)
option(BUILD_APPS "Build test applications" ON)

set(CMAKE_MODULE_PATH ${CMAKE_BINARY_DIR})

if(APPLE)
  set(basePoint @loader_path)
else()
  set(basePoint $ORIGIN)
endif()

include(GNUInstallDirs)
file(RELATIVE_PATH relDir
  ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}
  ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})

set(CMAKE_INSTALL_RPATH ${basePoint} ${basePoint}/${relDir})

# Find required packages
find_package(eigen REQUIRED)
if(BUILD_TESTS)
  find_package(gtest REQUIRED)
endif()

include(GenerateExportHeader)
add_subdirectory(src)

