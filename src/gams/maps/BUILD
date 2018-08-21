package(default_visibility = ["//visibility:public"])

cc_library(
    name = "maps",
    srcs = glob([
        "*.cpp",
        "*.h",
    ]),
    hdrs = ["Grid.h"],
    include_prefix = "gams/maps",
    deps = [
        "@gams//:gams_base",
        "@madara",
    ],
)
