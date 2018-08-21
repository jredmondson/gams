package(default_visibility = ["//visibility:public"])

cc_library(
    name = "variables",
    srcs = glob([
        "*.cpp",
        "*.h",
    ]),
    hdrs = [
        "AccentStatus.h",
        "Agent.h",
        "AlgorithmStatus.h",
        "PlatformStatus.h",
        "Region.h",
        "Self.h",
        "Sensor.h",
        "Swarm.h",
    ],
    include_prefix = "gams/variables",
    deps = [
        "@gams//:gams_base",
        "@gams//src/gams/groups",
        "@gams//src/gams/pose",
        "@gams//src/gams/utility",
        "@madara",
    ],
)
