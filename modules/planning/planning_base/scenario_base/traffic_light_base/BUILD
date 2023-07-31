load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])
PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

cc_library(
    name = "base_stage_traffic_light_creep",
    srcs = ["base_stage_traffic_light_creep.cc"],
    hdrs = ["base_stage_traffic_light_creep.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/map/pnc_map:path",
        "//modules/planning/planning_base/scenario_base:stage",
    ],
)

cc_library(
    name = "base_stage_traffic_light_cruise",
    srcs = ["base_stage_traffic_light_cruise.cc"],
    hdrs = ["base_stage_traffic_light_cruise.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning/planning_base/scenario_base:stage",
    ],
)

apollo_package()
cpplint()
