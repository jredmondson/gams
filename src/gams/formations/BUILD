package(default_visibility = ["//visibility:public"])

cc_library(
    name = "formations",
    srcs = glob([
        "*.cpp",
        "*.h",
    ]),
    hdrs = [
        "AgentFormation.h",
        "AgentFormation.inl",
        "StructuralFormation.h",
    ],
    include_prefix = "gams/formations",
    textual_hdrs = ["AgentFormation.inl"],
    deps = [
        "@gams//src/gams/groups",
        "@gams//src/gams/loggers",
        "@madara",
    ],
)
