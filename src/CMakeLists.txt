
# Note that headers are optional, and do not affect add_library, but they will not
# show up in IDEs unless they are listed in add_library.

cmake_minimum_required(VERSION 3.10)

if(CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

# Optionally glob, but only for CMake 3.12 or later:
# file(GLOB HEADER_LIST CONFIGURE_DEPENDS "${ModernCMakeExample_SOURCE_DIR}/include/modern/*.hpp")

set(header_path ${GAMS_SOURCE_DIR}/src/gams)

file(GLOB_RECURSE GAMS_SRC_FILES CONFIGURE_DEPENDS
    ${PROJECT_SOURCE_DIR}/src/gams/*.c
    ${PROJECT_SOURCE_DIR}/src/gams/*.cpp 
)

file(GLOB_RECURSE GAMS_HEADER_FILES CONFIGURE_DEPENDS
    ${PROJECT_SOURCE_DIR}/src/gams/*.h
    ${PROJECT_SOURCE_DIR}/src/gams/*.inl
)


set(EXCLUDE_DIR "/programs/")
foreach(TMP_PATH ${GAMS_SRC_FILES})
  string(FIND ${TMP_PATH} ${EXCLUDE_DIR} EXCLUDE_DIR_FOUND)
  if(NOT ${EXCLUDE_DIR_FOUND} EQUAL -1)
	  list(REMOVE_ITEM GAMS_SRC_FILES ${TMP_PATH})
  endif()
endforeach(TMP_PATH)

set(EXCLUDE_DIR "/plugins/")
foreach(TMP_PATH ${GAMS_SRC_FILES})
  string(FIND ${TMP_PATH} ${EXCLUDE_DIR} EXCLUDE_DIR_FOUND)
  if(NOT ${EXCLUDE_DIR_FOUND} EQUAL -1)
	  list(REMOVE_ITEM GAMS_SRC_FILES ${TMP_PATH})
  endif()
endforeach(TMP_PATH)

# remove all files in the java tree if it isn't feature-enabled
if(NOT JAVA)
  set(EXCLUDE_DIR "/java/")
  foreach(TMP_PATH ${GAMS_SRC_FILES})
      string(FIND ${TMP_PATH} ${EXCLUDE_DIR} EXCLUDE_DIR_FOUND)
      if(NOT ${EXCLUDE_DIR_FOUND} EQUAL -1)
          list(REMOVE_ITEM GAMS_SRC_FILES ${TMP_PATH})
      endif()
  endforeach(TMP_PATH)
  
  foreach(TMP_PATH ${GAMS_HEADER_FILES})
      string(FIND ${TMP_PATH} ${EXCLUDE_DIR} EXCLUDE_DIR_FOUND)
      if(NOT ${EXCLUDE_DIR_FOUND} EQUAL -1)
          list(REMOVE_ITEM GAMS_HEADER_FILES ${TMP_PATH})
      endif()
  endforeach(TMP_PATH)
endif()

# remove all files in the ros tree if it isn't feature-enabled
if(NOT ROS)
  set(EXCLUDE_DIR "/ros/")
  foreach(TMP_PATH ${GAMS_SRC_FILES})
      string(FIND ${TMP_PATH} ${EXCLUDE_DIR} EXCLUDE_DIR_FOUND)
      if(NOT ${EXCLUDE_DIR_FOUND} EQUAL -1)
          list(REMOVE_ITEM GAMS_SRC_FILES ${TMP_PATH})
      endif()
  endforeach(TMP_PATH)
  
  foreach(TMP_PATH ${GAMS_HEADER_FILES})
      string(FIND ${TMP_PATH} ${EXCLUDE_DIR} EXCLUDE_DIR_FOUND)
      if(NOT ${EXCLUDE_DIR_FOUND} EQUAL -1)
          list(REMOVE_ITEM GAMS_HEADER_FILES ${TMP_PATH})
      endif()
  endforeach(TMP_PATH)
endif()

if(MSVC)
  set(Boost_USE_STATIC_LIBS        ON)  # only find static libs
  set(Boost_USE_DEBUG_LIBS         ON) # ignore debug libs and
  set(Boost_USE_RELEASE_LIBS       ON)  # only find release libs
  set(Boost_USE_MULTITHREADED      ON)
  set(Boost_USE_STATIC_RUNTIME     ON)
  #find_package(Boost 1.60.0 REQUIRED)
else()
  set(Boost_USE_STATIC_LIBS        OFF)  # only find static libs
  set(Boost_USE_DEBUG_LIBS         OFF) # ignore debug libs and
  set(Boost_USE_RELEASE_LIBS       ON)  # only find release libs
  set(Boost_USE_MULTITHREADED      ON)
  set(Boost_USE_STATIC_RUNTIME     OFF)
  #find_package(Boost 1.60.0 REQUIRED COMPONENTS system)
endif()

# try to match up any GAMS features requested
set(madara_KARL               ${gams_KARL})
set(madara_PYTHON             ${gams_PYTHON})
set(madara_SIMTIME            ${gams_SIMTIME})
set(madara_THREADLOCAL        ${gams_THREADLOCAL})
set(madara_THREADLOCAL        ${gams_THREADLOCAL})

# We need this directory, and users of our library will need it too
#find_package(madara 3.3.0 REQUIRED)

add_custom_command(
  OUTPUT ${header_path}/Version.h
  COMMAND perl
  ARGS ${GAMS_BIN_DIR}/generate_version_include.pl
  DEPENDS ${GAMS_SOURCE_DIR}/VERSION.txt
)

add_custom_target(
  gams_lib_version
  DEPENDS ${header_path}/Version.h
)

if(madara_FOUND)
  
  add_library(gams SHARED
    ${GAMS_HEADER_FILES}
	  ${GAMS_SRC_FILES}) 

  add_dependencies(gams gams_lib_version)
	
  # target_include_directories(gams PUBLIC "$ENV{EIGEN_ROOT}")
  # target_include_directories(gams PUBLIC "$ENV{OSC_ROOT}")
  
  set_target_properties(gams PROPERTIES 
    DEBUG_POSTFIX d)

  target_include_directories(gams
    PUBLIC $<BUILD_INTERFACE:${GAMS_SOURCE_DIR}/src/>
         $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
         $<INSTALL_INTERFACE:${GAMS_INC_INSTALL_DIR}>
    PRIVATE ${header_path})
  
  target_compile_definitions(gams PRIVATE -DGAMS_BUILD_DLL)
  target_compile_definitions(gams PUBLIC -D_USE_MATH_DEFINES)

  if(gams_ANDROID)
    target_compile_definitions(gams PUBLIC -D_GAMS_ANDROID_)
  endif()

  if(gams_JAVA)
    target_compile_definitions(gams PUBLIC -D_GAMS_JAVA_)
  endif()

  if(gams_OSC)
    target_compile_definitions(gams PUBLIC -D_GAMS_OSC_)
  endif()

  if(gams_ROS)
    target_compile_definitions(gams PUBLIC -D_GAMS_ROS_)
  endif()

  # All users of this library will need at least C++14
  target_compile_features(gams PUBLIC cxx_std_14)
  
  if(MSVC)
    target_compile_options(gams PRIVATE /MP)
	
	  if(gams_OSC)
	  
	    target_link_libraries(gams PRIVATE
	      $<$<CONFIG:Debug>:$ENV{OSC_ROOT}/Debug/oscpack.lib>
	      $<$<CONFIG:Release>:$ENV{OSC_ROOT}/Release/oscpack.lib>
	    )
	  endif()
    target_link_libraries(gams PUBLIC Boost::boost)
  else()
	  if(gams_OSC)
	    target_link_libraries(gams PRIVATE oscpack)
    endif()
    target_link_libraries(gams PUBLIC Boost::boost Boost::system)
  endif()
  
  target_link_libraries(gams PUBLIC madara::madara)
  
  set_target_properties(gams PROPERTIES
    OUTPUT_NAME "gams"
    POSITION_INDEPENDENT_CODE ON)
									   
  install(TARGETS gams EXPORT gamsTargets
    RUNTIME       DESTINATION ${GAMS_RUNTIME_INSTALL_DIR}
    LIBRARY       DESTINATION ${GAMS_LIBRARY_INSTALL_DIR}
    ARCHIVE       DESTINATION ${GAMS_ARCHIVE_INSTALL_DIR}
    FRAMEWORK     DESTINATION ${GAMS_FRAMEWORK_INSTALL_DIR})

  # Write/install version file
  include(CMakePackageConfigHelpers)
  set(version_file "${CMAKE_CURRENT_BINARY_DIR}/cmake/gams-config-version.cmake")
  write_basic_package_version_file(${version_file}
    VERSION ${GAMS_VERSION}
    COMPATIBILITY AnyNewerVersion)
		
  #set_property(TARGET gams PROPERTY VERSION "${GAMS_VERSION}")
  set_property(TARGET gams PROPERTY SOVERSION "${GAMS_MAJOR_VERSION}.${GAMS_MINOR_VERSION}")

  install(TARGETS gams DESTINATION lib)
  install(DIRECTORY "${PROJECT_SOURCE_DIR}/src/gams" DESTINATION include)
  install(FILES ${version_file} DESTINATION ${GAMS_CMAKE_CONFIG_INSTALL_DIR})	 
else()
  print_all_variables()
  message(ERROR "trouble finding MADARA")

endif()
