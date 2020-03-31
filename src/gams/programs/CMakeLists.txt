
# compile applications/tools

gams_app(gams_controller gams_controller.cpp)
gams_app(gams_version gams_version.cpp)

install(TARGETS gams_controller DESTINATION bin)
install(TARGETS gams_version DESTINATION bin)
