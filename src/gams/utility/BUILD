package(default_visibility = ["//visibility:public"])

cc_library(
    name = "pose_utility",
    srcs = glob(
        [
            "*.cpp",
            "*.h",
        ],
        exclude = ["Containerize.cpp"],
    ),
    hdrs = glob(
        ["*.h"],
        exclude = ["Containerize.h"],
    ),
    include_prefix = "gams/utility",
    deps = [
        "@gams//:gams_base",
        "@gams//src/gams/loggers",
        "@gams//src/gams/pose",
        "@madara",
    ],
)

cc_library(
    name = "containerize",
    srcs = [
        "Containerize.cpp",
        "Containerize.h",
    ],
    hdrs = [
        "Containerize.h",
    ],
    include_prefix = "gams/utility",
    deps = [
        "@gams//:gams_base",
        "@gams//src/gams/loggers",
        "@madara",
    ],
)

cc_library(
    name = "utility",
    deps = [
        ":containerize",
        ":pose_utility",
    ],
)

cc_library(
    name = "utility_java",
    srcs = glob([
        "java/*.cpp",
        "java/*.h",
    ]),
    hdrs = glob(["java/*.h"]),
    defines = ["_GAMS_JAVA_"],
    include_prefix = "gams/utility",
    deps = [":utility"],
)
