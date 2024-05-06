load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_test", "apollo_package", "apollo_plugin")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "runtime_files",
    srcs = glob([
        "conf/**",
    ]),
)

apollo_plugin(
    name = "libvalet_parking_scenario.so",
    srcs = [
        "stage_approaching_parking_spot.cc",
        "stage_parking.cc",
        "valet_parking_scenario.cc",
    ],
    hdrs = [
        "stage_approaching_parking_spot.h",
        "stage_parking.h",
        "valet_parking_scenario.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    description = ":plugins.xml",
    deps = [
        "//cyber",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
        "//modules/planning/scenarios/valet_parking/proto:valet_parking_cc_proto",
    ],
)

apollo_cc_test(
    name = "valet_parking_scenario_test",
    size = "small",
    srcs = ["valet_parking_scenario_test.cc"],
    linkopts = ["-lgomp"],
    linkstatic = True,
    deps = [
        ":valet_parking_scenario_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "stage_approaching_parking_spot_test",
    size = "small",
    srcs = ["stage_approaching_parking_spot_test.cc"],
    linkopts = ["-lgomp"],
    linkstatic = True,
    deps = [
        ":valet_parking_scenario_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "stage_parking_test",
    size = "small",
    srcs = ["stage_parking_test.cc"],
    linkopts = ["-lgomp"],
    linkstatic = True,
    deps = [
        ":valet_parking_scenario_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_package()

cpplint()
