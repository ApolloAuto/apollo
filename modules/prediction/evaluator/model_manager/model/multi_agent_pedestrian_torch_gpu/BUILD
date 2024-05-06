load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_plugin")
load("//tools/platform:build_defs.bzl", "if_aarch64", "if_gpu")
load("//third_party/gpus:common.bzl", "gpu_library", "if_cuda", "if_rocm")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "runtime_files",
    srcs = glob([
        "conf/**",
        "data/**",
    ]),
)

apollo_plugin(
    name = "libmodel.so",
    srcs = ["multi_agent_pedestrian_torch_model.cc"],
    hdrs = ["multi_agent_pedestrian_torch_model.h"],
    description = ":plugins.xml",
    deps = [
        "//cyber",
        "//modules/prediction:apollo_prediction",
        "//modules/prediction/proto:prediction_conf_cc_proto",
    ] + if_gpu(
        [
            "@libtorch_gpu",
        ],
        ["@libtorch_cpu"],
    ),
)

apollo_package()

cpplint()
