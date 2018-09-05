load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "control_fuzz",
    srcs = [
        "control_fuzz.cc",
    ],
    copts = [
        "-fsanitize=fuzzer,address,undefined",
        "-Iexternal/libprotobuf_mutator/src/",
    ],
    linkopts = [
        "-fsanitize=fuzzer,address,undefined",
        "-lubsan",
    ],
    deps = [
        "//modules/control:control_lib",
        "//modules/tools/fuzz/control/proto:control_fuzz_proto",
        "@libprotobuf_mutator//:mutator",
    ],
)

cc_binary(
    name = "control_module_fuzz",
    srcs = [
        "control_module_fuzz.cc",
    ],
    copts = [
        "-fsanitize=fuzzer,address,undefined",
        "-Iexternal/libprotobuf_mutator/src/",
    ],
    linkopts = [
        "-fsanitize=fuzzer,address,undefined",
        "-lubsan",
    ],
    deps = [
        "//modules/control:control_lib",
        "//modules/tools/fuzz/control/proto:control_module_fuzz_proto",
        "@libprotobuf_mutator//:mutator",
    ],
)

cc_binary(
    name = "lat_controller_fuzz",
    srcs = [
        "lat_controller_fuzz.cc",
    ],
    copts = [
        "-fsanitize=fuzzer,address,undefined",
        "-Iexternal/libprotobuf_mutator/src/",
    ],
    linkopts = [
        "-fsanitize=fuzzer,address,undefined",
        "-lubsan",
    ],
    deps = [
        "//modules/common:log",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/control/common:control_gflags",
        "//modules/control/controller:lat_controller",
        "//modules/control/proto:control_proto",
        "//modules/planning/proto:planning_proto",
        "@libprotobuf_mutator//:mutator",
    ],
)

cc_binary(
    name = "lon_controller_fuzz",
    srcs = [
        "lon_controller_fuzz.cc",
    ],
    copts = [
        "-fsanitize=fuzzer,address,undefined",
        "-Iexternal/libprotobuf_mutator/src/",
    ],
    linkopts = [
        "-fsanitize=fuzzer,address,undefined",
        "-lubsan",
    ],
    deps = [
        "//modules/common:log",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/control/common:control_gflags",
        "//modules/control/controller:lon_controller",
        "//modules/control/proto:control_proto",
        "//modules/planning/proto:planning_proto",
        "@libprotobuf_mutator//:mutator",
    ],
)

cpplint()
