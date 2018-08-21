package(default_visibility = ["//visibility:public"])

cc_library(
    name = "base_area_coverage",
    srcs = [
        "BaseAreaCoverage.cpp",
        "BaseAreaCoverage.h",
    ],
    hdrs = ["BaseAreaCoverage.h"],
    include_prefix = "gams/algorithms/area_coverage",
    deps = ["@gams//src/gams/algorithms:base_algorithm"],
)

AREA_COVERAGE_FILES = [
    "LocalPheremoneAreaCoverage",
    "MinTimeAreaCoverage",
    "PerimeterPatrolCoverage",
    "PrioritizedMinTimeAreaCoverage",
    "PriorityWeightedRandomAreaCoverage",
    "SnakeAreaCoverage",
    "UniformRandomAreaCoverage",
    "UniformRandomEdgeCoverage",
    "WaypointsCoverage",
]

[cc_library(
    name = area_coverage,
    srcs = [
        area_coverage + ".cpp",
        area_coverage + ".h",
    ],
    hdrs = [area_coverage + ".h"],
    include_prefix = "gams/algorithms/area_coverage",
    deps = [":base_area_coverage"],
) for area_coverage in AREA_COVERAGE_FILES]

cc_library(
    name = "area_coverage",
    deps = [":" + area_coverage for area_coverage in AREA_COVERAGE_FILES],
)
