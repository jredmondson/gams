
# compile transport tests

gams_test(test_arguments_parser  test_arguments_parser.cpp)
gams_test(test_auctions          test_auctions.cpp)
gams_test(test_controller        test_controller.cpp)
gams_test(test_coordinates       test_coordinates.cpp)
gams_test(test_elections         test_elections.cpp)
gams_test(test_euler             test_euler.cpp)
gams_test(test_formation_sync    test_formation_sync.cpp)
gams_test(test_groups            test_groups.cpp)
gams_test(test_joystick          test_joystick.cpp)
gams_test(test_location          test_location.cpp)
gams_test(test_multicontroller   test_multicontroller.cpp)
gams_test(test_utility           test_utility.cpp)
gams_test(test_utm               test_utm.cpp)
gams_test(test_variables         test_variables.cpp)

if(gams_ROS)
  gams_test(test_types           test_types.cpp)
  gams_test(test_ros2gams        test_ros2gams.cpp)
endif()

# probably a macro or function I should do for multifiles, but there
# are only 2 tests I needed that for, so these are slightly more verbose
add_executable(test_controller_run
  test_controller_run.cpp
  helper/CounterAlgorithm.h
  helper/CounterAlgorithm.cpp
  helper/CounterPlatform.h
  helper/CounterPlatform.cpp)
target_link_libraries(test_controller_run PRIVATE gams)

add_executable(gams_throughput
  throughput/gams_throughput.cpp
  throughput/algorithms/Modify.h
  throughput/algorithms/Modify.cpp
  throughput/filters/RcvCount.h
  throughput/filters/RcvCount.cpp
  throughput/utility/MovingAverage.h
  throughput/utility/MovingAverage.cpp)
target_link_libraries(gams_throughput PRIVATE gams)
    
install(TARGETS gams_throughput EXPORT gamsTargets
           RUNTIME DESTINATION ${MADARA_RUNTIME_INSTALL_DIR})
