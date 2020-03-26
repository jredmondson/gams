
macro(gams_app _NAME _FILE)
  add_executable(${_NAME} ${_FILE})
  target_link_libraries(${_NAME} PRIVATE gams)
endmacro()

macro(gams_test _NAME _FILE)
  add_executable(${_NAME} ${_FILE})
  target_link_libraries(${_NAME} PRIVATE gams)
endmacro()
