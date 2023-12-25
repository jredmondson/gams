
cmake_minimum_required(VERSION 3.10)

# Read VERSION.txt and set GAMS_VERSION variables
file(STRINGS "VERSION.txt" GAMS_VERSION)
string(REPLACE "." ";" VERSION_LIST ${GAMS_VERSION})
list(GET VERSION_LIST 0 GAMS_MAJOR_VERSION)
list(GET VERSION_LIST 1 GAMS_MINOR_VERSION)
list(GET VERSION_LIST 2 GAMS_PATCH_VERSION)

set(MIN_MADARA_VERSION  3.3.0)

# Get helper cmake macros
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(gams_macros)
  
# Define the GAMS project and version
project(GAMS VERSION "${GAMS_VERSION}")

# set(GAMS_ROOT ${PROJECT_SOURCE_DIR})

find_package(madara ${MIN_MADARA_VERSION} REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})

# Provide a lowercase version of project
string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)
string(TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPER)

set(GAMS_CMAKE_DIR ${PROJECT_SOURCE_DIR}/cmake CACHE PATH "Location of CMake scripts")
set(GAMS_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/src CACHE PATH "Location of GAMS source")
set(GAMS_BIN_DIR ${PROJECT_SOURCE_DIR}/bin CACHE PATH "Location of GAMS scripts and binaries")
set(GAMS_LIB_DIR ${PROJECT_SOURCE_DIR}/lib CACHE PATH "Location of GAMS libraries")

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
    

# specify the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# disable certain warnings that will occur in VS
if(MSVC)

  set(GAMS_INC_INSTALL_DIR "include")
  set(GAMS_RUNTIME_INSTALL_DIR   "bin")
  set(GAMS_LIBRARY_INSTALL_DIR   "bin")
  set(GAMS_ARCHIVE_INSTALL_DIR   "lib")
  set(GAMS_FRAMEWORK_INSTALL_DIR "bin")

  set(GAMS_CMAKE_CONFIG_INSTALL_DIR "share/gams/cmake")
  set(GAMS_ADDITIONAL_FILES_INSTALL_DIR "share/gams")

  add_compile_options(/wd4005)
  add_compile_options(/wd4251)
  add_compile_options(/wd4275)
  add_compile_options(/wd4244)
  add_compile_options(/wd4297)
  add_compile_options(/wd4996)
  add_compile_options(/MP)
  
# make linux/mac compilation stricter
elseif(UNIX)
  include(GNUInstallDirs)

  set(GAMS_INC_INSTALL_DIR "${CMAKE_INSTALL_INCLUDEDIR}/gams")
  set(GAMS_RUNTIME_INSTALL_DIR "${CMAKE_INSTALL_BINDIR}")
  set(GAMS_LIBRARY_INSTALL_DIR "${CMAKE_INSTALL_LIBDIR}")
  set(GAMS_ARCHIVE_INSTALL_DIR "${CMAKE_INSTALL_LIBDIR}")
  set(GAMS_FRAMEWORK_INSTALL_DIR "${CMAKE_INSTALL_LIBDIR}")

  set(GAMS_CMAKE_CONFIG_INSTALL_DIR "${CMAKE_INSTALL_LIBDIR}/gams/cmake")
  set(GAMS_ADDITIONAL_FILES_INSTALL_DIR "${CMAKE_INSTALL_DATADIR}/gams")

  set(GAMS_RUNTIME_INSTALL_DIR "bin")
  set(GAMS_INC_INSTALL_DIR "include/gams") # header filestree

  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")
endif()
	
include(cmake/configuration.cmake)

# Build the main library
add_subdirectory(src)
#add_subdirectory(src/gams/programs)

# Build tests
if(gams_TESTS)
  set(GAMS_ROOT "${PROJECT_SOURCE_DIR}")
  enable_testing()
  add_subdirectory(tests)
endif()

if(gams_DOCS)
  add_subdirectory(docs)
endif()


set(PKG_NAME ${PROJECT_NAME_UPPER})
set(PKG_LIBRARIES gams)
set(PKG_DEPENDS madara Boost::boost Boost::system)

# install readme and license
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE.txt" "${CMAKE_CURRENT_SOURCE_DIR}/VERSION.txt" "${CMAKE_CURRENT_SOURCE_DIR}/README.md" DESTINATION ${GAMS_ADDITIONAL_FILES_INSTALL_DIR})

install(EXPORT gamsTargets NAMESPACE gams:: 
  DESTINATION ${GAMS_CMAKE_CONFIG_INSTALL_DIR}
  FILE gams-config.cmake)
  