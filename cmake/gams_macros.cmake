
macro(gams_app _NAME _FILE)
  add_executable(${_NAME} ${_FILE})
  target_link_libraries(${_NAME} PRIVATE gams)
  
  install(TARGETS ${_NAME} EXPORT gamsTargets
           RUNTIME DESTINATION ${GAMS_RUNTIME_INSTALL_DIR})

endmacro()

macro(gams_test _NAME _FILE)
  add_executable(${_NAME} ${_FILE})
  target_link_libraries(${_NAME} PRIVATE gams)
  
  install(TARGETS ${_NAME} EXPORT gamsTargets
           RUNTIME DESTINATION ${GAMS_RUNTIME_INSTALL_DIR})

endmacro()

macro(print_all_variables)
    message(STATUS "print_all_variables------------------------------------------{")
    get_cmake_property(_variableNames VARIABLES)
    foreach (_variableName ${_variableNames})
        message(STATUS "${_variableName}=${${_variableName}}")
    endforeach()
    message(STATUS "print_all_variables------------------------------------------}")
endmacro()