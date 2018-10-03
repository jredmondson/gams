package(default_visibility = ["//visibility:public"])

# Python binding support
cc_binary(
    name = "gams.so",
    srcs = glob(
        [
            "src/*.h",
            "src/*.cpp",
        ],
    ),
    copts = [
        "-w",
    ],
    linkshared = 1,
    deps = [
        "@gams//:gams_python",
    ],
)

py_library(
    name = "gams",
    data = [
        ":gams.so",
    ],
    imports = ["."],
)
