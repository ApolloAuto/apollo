load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

install(
    name = "install",
    data_dest = "common/vehicle_model",
    library_dest = "common/lib",
    runtime_dest = "common/bin",
    data = [
        ":vehicle_model_config_data",
    ],
    targets = [
        ":libvehicle_model.so",
    ],
    visibility = ["//visibility:public"],
)


cc_binary(
    name = "libvehicle_model.so",
    srcs = [
        "vehicle_model.cc",
        "vehicle_model.h",
    ],
    linkshared = True,
    linkstatic = True,
    deps = [
        "//cyber",
        "//modules/common/configs:config_gflags",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/vehicle_model/proto:vehicle_model_config_cc_proto",
        "//modules/common/vehicle_state/proto:vehicle_state_cc_proto",
    ],
)

cc_library(
    name = "vehicle_model",
    srcs = ["libvehicle_model.so"],
    hdrs = ["vehicle_model.h"],
    deps = [
        "//cyber",
        "//modules/common/configs:config_gflags",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/vehicle_model/proto:vehicle_model_config_cc_proto",
        "//modules/common/vehicle_state/proto:vehicle_state_cc_proto",
    ],
    visibility = ["//visibility:public"],
)

cc_test(
    name = "vehicle_model_test",
    size = "small",
    srcs = ["vehicle_model_test.cc"],
    data = [
        ":test_data",
    ],
    deps = [
        ":vehicle_model",
        "//cyber",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "@com_google_googletest//:gtest_main",
    ],
)

filegroup(
    name = "test_data",
    srcs = glob([
        "testdata/*.txt",
    ]),
)

filegroup(
    name = "vehicle_model_config_data",
    srcs = ["conf/vehicle_model_config.pb.txt"],
)
cpplint()
