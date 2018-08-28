package(default_visibility = ["//visibility:public"])

cc_library(
    name = "elections",
    srcs = glob([
        "*.cpp",
        "*.h",
    ]),
    hdrs = [
        "ElectionBase.h",
        "ElectionBase.inl",
        "ElectionCumulative.h",
        "ElectionFactory.h",
        "ElectionFactoryRepository.h",
        "ElectionFactoryRepository.inl",
        "ElectionPlurality.h",
        "ElectionTypesEnum.h",
    ],
    include_prefix = "gams/elections",
    textual_hdrs = [
        "ElectionBase.inl",
        "ElectionFactoryRepository.inl",
    ],
    deps = [
        "@gams//:gams_base",
        "@gams//src/gams/groups",
        "@gams//src/gams/loggers",
        "@madara",
    ],
)
