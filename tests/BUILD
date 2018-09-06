load("@bazel_module//bazel_rules:auxiliary.bzl", "basename", "map")

DISABLED_CPP_TESTS = [
    "test_vrep_controller",
    "test_ros2gams",
    "test_controller_run",
    "test_types",
]

cc_library(
    name = "test_helpers",
    srcs = glob([
        "helper/*.cpp",
        "helper/*.h",
    ]),
    hdrs = glob(["helper/*.h"]),
    linkstatic = 1,
    deps = ["@gams"],
)

[cc_test(
    name = f,
    size = "small",
    srcs = [f + ".cpp"],
    deps = [
        ":test_helpers",
        "@gams",
    ],
) for f in map(
    basename,
    glob(["*.cpp"]),
) if f not in DISABLED_CPP_TESTS]

# test_controller_ruun is a long test (takes around 5 minutes to finish)
cc_test(
    name = "test_controller_run",
    timeout = "long",
    srcs = ["test_controller_run.cpp"],
    deps = [
        ":test_helpers",
        "@gams",
    ],
)
