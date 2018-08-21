package(default_visibility = ["//visibility:public"])

cc_library(
    name = "base_algorithm",
    srcs = [
        "AlgorithmFactory.cpp",
        "AlgorithmFactory.h",
        "BaseAlgorithm.cpp",
        "BaseAlgorithm.h",
    ],
    hdrs = [
        "AlgorithmFactory.h",
        "BaseAlgorithm.h",
    ],
    include_prefix = "gams/algorithms",
    deps = [
        "@gams//src/gams/loggers",
        "@gams//src/gams/platforms",
        "@gams//src/gams/pose",
        "@gams//src/gams/variables",
    ],
)

ALGORITHM_FILES = [
    "AlgorithmFactoryRepository",
    "Hold",
    "Spell",
    "Land",
    "Move",
    "DebugAlgorithm",
    "NullAlgorithm",
    "FormationCoverage",
    "FormationSync",
    "KarlEvaluator",
    "ZoneCoverage",
    "Takeoff",
    "Home",
    "Follow",
    "MessageProfiling",
    "PerimeterPatrol",
    "Executor",
    "Wait",
    "PerformanceProfiling",
    "GroupBarrier",
    "FormationFlying",
]

cc_library(
    name = "algorithms",
    srcs = [file + filetype for file in ALGORITHM_FILES for filetype in [
        ".cpp",
        ".h",
    ]],
    hdrs = [file + ".h" for file in ALGORITHM_FILES],
    include_prefix = "gams/algorithms",
    deps = [
        ":base_algorithm",
        "@gams//src/gams/algorithms/area_coverage",
        "@gams//src/gams/loggers",
        "@madara",
    ],
)

cc_library(
    name = "algorithms_java",
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
    include_prefix = "gams/algorithms",
    deps = [
        ":algorithms",
        "@gams//:gams_jni_h",
        "@gams//src/gams/utility:utility_java",
    ],
)
