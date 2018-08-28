package(default_visibility = ["//visibility:public"])

cc_library(
    name = "groups",
    srcs = glob([
        "*.cpp",
        "*.h",
    ]),
    hdrs = [
        "GroupBase.h",
        "GroupBase.inl",
        "GroupFactory.h",
        "GroupFactoryRepository.h",
        "GroupFactoryRepository.inl",
        "GroupFixedList.h",
        "GroupTransient.h",
        "GroupTypesEnum.h",
    ],
    include_prefix = "gams/groups",
    textual_hdrs = [
        "GroupBase.inl",
        "GroupFactoryRepository.inl",
    ],
    deps = [
        "@gams//:gams_base",
        "@gams//src/gams/loggers",
        "@madara",
    ],
)
