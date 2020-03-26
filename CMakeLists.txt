cmake_minimum_required(VERSION 3.12)

# Read VERSION.txt and set GAMS_VERSION variables
file(STRINGS "VERSION.txt" GAMS_VERSION)
string(REPLACE "." ";" VERSION_LIST ${GAMS_VERSION})
list(GET VERSION_LIST 0 GAMS_MAJOR_VERSION)
list(GET VERSION_LIST 1 GAMS_MINOR_VERSION)
list(GET VERSION_LIST 2 GAMS_PATCH_VERSION)

# Get helper cmake macros
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(gams_macros)
  
# Define the GAMS project and version
project(GAMS VERSION "${GAMS_VERSION}")

# Provide a lowercase version of project
string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)
string(TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPER)

set(GAMS_CMAKE_DIR ${PROJECT_SOURCE_DIR}/cmake CACHE PATH "Location of CMake scripts")
set(GAMS_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include CACHE PATH "Location of GAMS source")
set(GAMS_BIN_DIR ${PROJECT_SOURCE_DIR}/bin CACHE PATH "Location of GAMS scripts and binaries")
set(GAMS_LIB_DIR ${PROJECT_SOURCE_DIR}/lib CACHE PATH "Location of GAMS libraries")

set(CMAKE_DEBUG_POSTFIX d)

########################################
# Package Creation: 
include (${GAMS_CMAKE_DIR}/gams_cpack.cmake)
set (CPACK_PACKAGE_VERSION "${GAMS_VERSION}")
set (CPACK_PACKAGE_VERSION_MAJOR "${GAMS_MAJOR_VERSION}")
set (CPACK_PACKAGE_VERSION_MINOR "${GAMS_MINOR_VERSION}")
set (CPACK_PACKAGE_VERSION_PATCH "${GAMS_PATCH_VERSION}")

if (CPACK_GENERATOR)
  message(STATUS "Found CPack generators: ${CPACK_GENERATOR}")

  configure_file("${GAMS_CMAKE_DIR}/cpack_options.cmake.in" ${GAMS_CPACK_CFG_FILE} @ONLY)
  set(CPACK_PROJECT_CONFIG_FILE ${GAMS_CPACK_CFG_FILE})
  include (CPack)
endif()


# Use GNUInstallDirst to get canonical paths
include(GNUInstallDirs)

set (CMAKE_INCLUDE_DIRECTORIES_PROJECT_BEFORE ON)


# developer's option to cache PKG_CONFIG_PATH and
# LD_LIBRARY_PATH for local installs
if(PKG_CONFIG_PATH)
  set (ENV{PKG_CONFIG_PATH} ${PKG_CONFIG_PATH}:$ENV{PKG_CONFIG_PATH})
endif()
if(LD_LIBRARY_PATH)
  set (ENV{LD_LIBRARY_PATH} ${LD_LIBRARY_PATH}:$ENV{LD_LIBRARY_PATH})
endif()

set (INCLUDE_INSTALL_DIR "${CMAKE_INSTALL_INCLUDEDIR}/gams-${GAMS_VERSION}/gams") 
set (LIB_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR})
set (BIN_INSTALL_DIR ${CMAKE_INSTALL_BINDIR})
    

# Options
option(ANDROID "Build with Android support" OFF)
option(DOCS "Run doxygen or javadocs on code" OFF)
option(KARL "Build with KaRL scripting language support (eval/wait)" ON)
option(OSC "Build with OSC support" ON)
option(PYTHON "Build with python support (via Boost.Python)" OFF)
option(ROS "Build with ROS support" OFF)
option(SIMTIME "Build with support for simulation/virtual time" OFF)
option(TESTS "Build tests" OFF)
option(THREADLOCAL "Build with support for threadlocal variables" OFF)
option(XML "Build with XML library support" OFF)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_custom_command(
  OUTPUT ${GAMS_INCLUDE_DIR}/Version.h
  COMMAND ${GAMS_BIN_DIR}/generate_version_include.pl
)

if(ANDROID)
  add_definitions(-D_GAMS_ANDROID_ -D__STDC_FORMAT_MACROS)
endif()

if(JAVA)
  add_definitions(-D_GAMS_JAVA_)
endif()

if(NOT KARL)
  add_definitions(-D_MADARA_NO_KARL_)
endif()

if(OSC)
endif()

if(PYTHON)
  add_definitions(-D_MADARA_PYTHON_CALLBACKS_)
endif()

if(ROS)
  add_definitions(-D_GAMS_ROS_)
endif()

if(SIMTIME)
  add_definitions(-DMADARA_FEATURE_SIMTIME)
endif()

if(THREADLOCAL)
  add_definitions(-DMADARA_THREAD_LOCAL)
else()
  add_definitions(-DMADARA_NO_THREAD_LOCAL)
endif()

# disable certain warnings that will occur in VS
if(MSVC)
  add_compile_options(/wd4005)
  add_compile_options(/wd4251)
  add_compile_options(/wd4275)
  add_compile_options(/wd4244)
  add_compile_options(/wd4297)
  add_compile_options(/wd4996)
  add_compile_options(/MP)
  
  
# make linux/mac compilation stricter
elseif(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")
endif()
	
# Build the main library
add_subdirectory(src)
add_subdirectory(src/gams/programs)

# Build tests
if(TESTS)
  add_subdirectory(tests)
endif()

if(DOCS)
  add_subdirectory(docs)
endif()


set(PKG_NAME ${PROJECT_NAME_UPPER})
set(PKG_LIBRARIES gams)
set(PKG_DEPENDS madara)

install(FILES "${PROJECT_SOURCE_DIR}/VERSION.txt" DESTINATION .)
install(FILES "${PROJECT_SOURCE_DIR}/LICENSE.txt" DESTINATION .)
