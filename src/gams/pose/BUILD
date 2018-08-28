package(default_visibility = ["//visibility:public"])

cc_library(
    name = "pose",
    srcs = glob([
        "*.cpp",
        "**/*.h",
    ]),
    hdrs = glob([
        "*.h",
        "*.inl",
    ]),
    include_prefix = "gams/pose",
    textual_hdrs = glob(["*.inl"]),
    deps = [
        "@com_bitbucket_eigen3//:eigen",
        "@gams//:gams_base",
        "@gams//src/gams/exceptions",
        "@gams//src/gams/loggers",
        "@gams//src/gams/utility:containerize",
        "@madara",
    ],
)
