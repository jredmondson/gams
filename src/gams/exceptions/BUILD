package(default_visibility = ["//visibility:public"])

cc_library(
    name = "exceptions",
    srcs = glob([
        "*.cpp",
        "**/*.h",
    ]),
    hdrs = glob([
        "*.h",
        "*.inl",
    ]),
    include_prefix = "gams/exceptions",
    textual_hdrs = glob(["*.inl"]),
    deps = [
        "@gams//:gams_base",
    ],
)
