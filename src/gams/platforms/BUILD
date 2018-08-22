package(default_visibility = ["//visibility:public"])

cc_library(
    name = "platforms",
    srcs = glob(
        [
            "*.cpp",
            "*.h",
        ],
        exclude = [
            "java/*",
            "vrep/*",
            "ros/*",
        ],
    ),
    hdrs = [
        "BasePlatform.h",
        "BasePlatform.inl",
        "DebugPlatform.h",
        "NullPlatform.h",
        "PlatformFactory.h",
        "PlatformFactoryRepository.h",
    ],
    include_prefix = "gams/platforms",
    textual_hdrs = ["BasePlatform.inl"],
    deps = [
        "@gams//src/gams/pose",
        "@gams//src/gams/utility",
        "@gams//src/gams/variables",
        "@madara",
    ],
)

cc_library(
    name = "platforms_java",
    srcs = glob([
        "java/*.cpp",
        "java/*.h",
    ]),
    hdrs = glob(["java/*.h"]),
    copts = [
        "-I external/local_jdk/include",
        "-I external/local_jdk/include/linux",
    ],
    defines = ["_GAMS_JAVA_"],
    include_prefix = "gams/platforms",
    deps = [
        ":platforms",
        "@gams//:gams_jni_h",
        "@gams//src/gams/utility:utility_java",
    ],
)
