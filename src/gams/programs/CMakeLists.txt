
# compile applications/tools

gams_app(gams_controller gams_controller.cpp)

install(TARGETS gams_controller DESTINATION bin)
