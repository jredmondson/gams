
# Note that headers are optional, and do not affect add_library, but they will not
# show up in IDEs unless they are listed in add_library.

cmake_minimum_required(VERSION 3.12)

# Optionally glob, but only for CMake 3.12 or later:
# file(GLOB HEADER_LIST CONFIGURE_DEPENDS "${ModernCMakeExample_SOURCE_DIR}/include/modern/*.hpp")

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

# remove all files in the splice tree if it isn't feature-enabled
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

# remove all files in the splice tree if it isn't feature-enabled
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


# Make an automatic library - will be static or dynamic based on user setting

set(Boost_USE_STATIC_LIBS        ON)  # only find static libs
set(Boost_USE_DEBUG_LIBS         ON) # ignore debug libs and
set(Boost_USE_RELEASE_LIBS       ON)  # only find release libs
set(Boost_USE_MULTITHREADED      ON)
set(Boost_USE_STATIC_RUNTIME     ON)
find_package(Boost 1.73.0) # REQUIRED COMPONENTS filesystem) 


# We need this directory, and users of our library will need it too
find_library(MADARA_LIB madara)

if(MADARA_LIB AND Boost_FOUND)
  
  add_library(gams SHARED
    ${GAMS_HEADER_FILES}
	${GAMS_SRC_FILES}) 
	
  #target_include_directories(gams PUBLIC ${Boost_INCLUDE_DIRS})
  target_include_directories(gams PUBLIC "$ENV{EIGEN_ROOT}")
  target_include_directories(gams PUBLIC "$ENV{OSC_ROOT}")
  target_include_directories(gams PUBLIC "$ENV{MADARA_ROOT}/include")
  target_include_directories(gams PUBLIC ${Boost_INCLUDE_DIRS})
  target_include_directories(gams PUBLIC ${PROJECT_SOURCE_DIR}/src)
  target_compile_definitions(gams PRIVATE -DGAMS_BUILD_DLL)
  target_compile_definitions(gams PUBLIC -D_WINSOCK_DEPRECATED_NO_WARNINGS -DBOOST_ALL_NO_LIB -D_USE_MATH_DEFINES)

  # All users of this library will need at least C++11
  target_compile_features(gams PUBLIC cxx_std_14)
  
  if(MSVC)
    target_compile_options(gams PRIVATE /MP)
	
	if(OSC)
	  if(CMAKE_BUILD_TYPE MATCHES DEBUG)
        target_link_libraries(gams PRIVATE "$ENV{OSC_ROOT}/Debug/oscpack.lib")
	  else()
        target_link_libraries(gams PRIVATE "$ENV{OSC_ROOT}/Release/oscpack.lib")
	  endif()
	endif()
  endif()
  
  target_link_libraries(gams PUBLIC "${MADARA_LIB}")
  
endif()

# IDEs should put the headers in a nice place
#source_group(TREE "${PROJECT_SOURCE_DIR}/include" PREFIX "Header Files" FILES ${GAMS_HEADER_FILES})

set_property(TARGET gams PROPERTY VERSION "${GAMS_VERSION}")
set_property(TARGET gams PROPERTY SOVERSION "${GAMS_VERSION}")

install(TARGETS gams DESTINATION lib)
INSTALL( DIRECTORY "${PROJECT_SOURCE_DIR}/src/gams" DESTINATION include )

#install(FILES ${GAMS_HEADER_FILES} DESTINATION include)
