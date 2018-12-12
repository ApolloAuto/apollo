load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "record_service",
    srcs = ["record_service.cc"],
    hdrs = ["record_service.h"],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util",
        "//modules/data/common:data_gflags",
        "//modules/data/proto:record_request_proto",
        "//modules/data/proto:record_response_proto",
        "//modules/data/proto:static_info_proto",
    ],
)

cpplint()
