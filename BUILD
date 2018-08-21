package(default_visibility = ["//visibility:public"])

cc_library(
    name = "gams",
    copts = ["-w"],
    deps = [
        "@gams//src/gams/algorithms",
        "@gams//src/gams/algorithms/area_coverage",
        "@gams//src/gams/auctions",
        "@gams//src/gams/controllers",
        "@gams//src/gams/elections",
        "@gams//src/gams/formations",
        "@gams//src/gams/groups",
        "@gams//src/gams/loggers",
        "@gams//src/gams/platforms",
        "@gams//src/gams/pose",
        "@gams//src/gams/utility",
        "@gams//src/gams/variables",
        "@madara",
    ],
)

cc_library(
    name = "gams_base",
    hdrs = [
        "src/gams/CPP11_compat.h",
        "src/gams/GamsExport.h",
        "src/gams/Namespaces.h",
    ],
    copts = ["-w"],
    strip_include_prefix = "src",
)

cc_library(
    name = "jni_headers",
    srcs = [
        "@local_jdk//:jni_header",
        "@local_jdk//:jni_md_header-linux",
    ],
    hdrs = [
        "@local_jdk//:jni_header",
        "@local_jdk//:jni_md_header-linux",
    ],
    copts = ["-w"],
    linkstatic = 1,
    visibility = [
        "//visibility:public",
    ],
)

cc_library(
    name = "gams_jni_h",
    srcs = glob(["port/java/jni/*.h"]),
    hdrs = glob(["port/java/jni/*.h"]),
    copts = ["-w"],
    includes = [
        "external/local_jdk/include/linux",
        "external/local_jdk/inlude",
    ],
    strip_include_prefix = "port/java/jni",
    deps = ["jni_headers"],
)

cc_library(
    name = "gams_jni",
    copts = ["-w"],
    linkstatic = 1,
    deps = [
        "@gams//src/gams/algorithms",
        "@gams//src/gams/algorithms/area_coverage",
        "@gams//src/gams/auctions",
        "@gams//src/gams/controllers:controllers_java",
        "@gams//src/gams/elections",
        "@gams//src/gams/formations",
        "@gams//src/gams/groups",
        "@gams//src/gams/loggers",
        "@gams//src/gams/platforms",
        "@gams//src/gams/platforms:platforms_java",
        "@gams//src/gams/pose",
        "@gams//src/gams/utility",
        "@gams//src/gams/variables",
        "@madara",
    ],
)

# Shared binary with the JNI support
cc_binary(
    name = "libGAMS.so",
    srcs = glob([
        "port/java/jni/*.cpp",
        "port/java/jni/*.h",
    ]) + [
        "@local_jdk//:jni_header",
        "@local_jdk//:jni_md_header-linux",
    ],
    copts = [
        "-I external/local_jdk/include",
        "-I external/local_jdk/include/linux",
        "-D_GAMS_JAVA_",
        "-w",
    ],
    linkshared = 1,
    deps = [
        ":gams_jni",
        ":gams_jni_h",
    ],
)

java_library(
    name = "gams_java",
    srcs = glob(["port/java/src/main/java/ai/gams/**/*.java"]),
    deps = ["@madara//:madara_java"],
)

#load("@bazel_module//bazel_rules:auxiliary.bzl", "map", "basename")
#DISABLED_JAVA_TESTS = ["TestDebuggerLoop",
#                       "TestMessagingThroughput",
#                       "TestMultiController",
#                       "TestUtility"]
#
#[java_test(
#    name = f,
#    main_class = "ai.gams.tests." + f,
#    runtime_deps = [
#       ":gams_java", "@madara//:madara_java", "@madara//:libMADARA.so", "@ace//:ace", ":libGAMS.so",
#    ],
#    use_testrunner = 0
#) for f in map(basename, glob(["port/java/src/main/java/ai/gams/tests/*.java"])) if f not in DISABLED_JAVA_TESTS]
