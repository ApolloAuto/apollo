load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")

package(default_visibility = ["//visibility:public"])

apollo_cc_binary(
    name = "map_datachecker_client",
    srcs = ["main.cc"],
    deps = [
        ":apollo_map_tools_map_datachecker_client",
        "@boost",
        "@com_github_grpc_grpc//:grpc++",
    ],
)

apollo_cc_library(
    name = "apollo_map_tools_map_datachecker_client",
    srcs = [
        "client.cc",
        "client_channel_checker.cc",
        "client_common.cc",
        "client_gflags.cc",
        "client_loops_check.cc",
        "exception_handler.cc",
    ],
    hdrs = [
        "client.h",
        "client_alignment.h",
        "client_channel_checker.h",
        "client_common.h",
        "client_gflags.h",
        "client_loops_check.h",
        "exception_handler.h",
    ],
    deps = [
        "//cyber",
        "//modules/map/tools/map_datachecker/proto:collection_error_code_cc_proto",
        "//modules/map/tools/map_datachecker/proto:collection_service_cc_grpc",
        "@boost",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_grpc_grpc//:grpc++",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
    ],
)

apollo_package()

cpplint()
