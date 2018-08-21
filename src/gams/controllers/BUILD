package(default_visibility = ["//visibility:public"])

cc_library(
    name = "controllers",
    srcs = glob([
        "*.cpp",
        "*.h",
    ]),
    hdrs = glob(["*.h"]),
    include_prefix = "gams/controllers",
    deps = [
        "@gams//:gams_base",
        "@gams//src/gams/algorithms",
        "@gams//src/gams/groups",
        "@gams//src/gams/platforms",
        "@gams//src/gams/variables",
        "@madara",
    ],
)

# controllers_java can't depend on controllers since
# the sources are not separated, but the jni bits
# are hidden behind the preprocesor directive, so
# it repeates the srcs and headers.
# Warning: do not depend both on controllers and
# controllers_java, that leads to the multiple defined
# references during the link time.
cc_library(
    name = "controllers_java",
    srcs = glob([
        "*.cpp",
        "*.h",
    ]),
    hdrs = glob(["*.h"]),
    copts = [
        "-I external/local_jdk/include",
        "-I external/local_jdk/include/linux",
    ],
    defines = ["_GAMS_JAVA_"],
    include_prefix = "gams/controllers",
    deps = [
        "@gams//src/gams/algorithms:algorithms_java",
        "@gams//src/gams/platforms:platforms_java",
    ],
)
