load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package", "apollo_cc_test")
load("@local_config_cuda//cuda:build_defs.bzl", "cuda_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cuda_library(
    name = "apollo_perception_common_inference_tensorrt_plugins",
    alwayslink = True,
    srcs = [
        "plugins/kernels.cu",
        "plugins/nms_cuda.cu",
        "plugins/dfmb_psroi_align_plugin.cu",
        "plugins/rcnn_proposal_plugin.cu",
        "plugins/rpn_proposal_ssd_plugin.cu",
        "plugins/slice_plugin.cu",
        "plugins/argmax_plugin.cu",
        "plugins/softmax_plugin.cu",
        "plugins/leakyReLU_plugin.cu"
    ],
    hdrs = [
        "plugins/kernels.h",
        "plugins/dfmb_psroi_align_plugin.h",
        "plugins/rcnn_proposal_plugin.h",
        "plugins/rpn_proposal_ssd_plugin.h",
        "plugins/slice_plugin.h",
        "plugins/argmax_plugin.h",
        "plugins/softmax_plugin.h",
        "plugins/leakyReLU_plugin.h",
    ],
    deps = [
        "@eigen",
        "@local_config_cuda//cuda:cublas",
        "@local_config_cuda//cuda:cudart",
        "@local_config_cuda//cuda:cudnn",
        "@local_config_tensorrt//:tensorrt",
        "//cyber",
        "//modules/perception/common/base:apollo_perception_common_base",
        ":apollo_perception_inference_rt_common",
    ],
)

apollo_cc_test(
    name = "slice_plugin_test",
    size = "small",
    srcs = ["plugins/slice_plugin_test.cc"],
    deps = [
        ":apollo_perception_common_inference_tensorrt_plugins",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "argmax_plugin_test",
    size = "small",
    srcs = ["plugins/argmax_plugin_test.cc"],
    deps = [
        ":apollo_perception_common_inference_tensorrt_plugins",
        "@com_google_googletest//:gtest_main",
        "@local_config_cuda//cuda:cublas",
    ],
    linkstatic = True,
)

apollo_cc_library(
    name = "apollo_perception_inference_rt_common",
    hdrs = [
        "rt_legacy.h",
        "rt_common.h",
        "rt_utils.h",
        "batch_stream.h",
        "entropy_calibrator.h",
    ],
    srcs = [
        "rt_common.cc",
        "batch_stream.cc",
        "rt_utils.cc",
        "entropy_calibrator.cc",
    ],
    deps = [
        "//cyber",
        "//modules/perception/common/base:apollo_perception_common_base",
        "@local_config_cuda//cuda:cudnn_header",
        "@local_config_tensorrt//:tensorrt",
        "//modules/perception/common/proto:rt_cc_proto",
        "//modules/perception/common/inference:inference_lib",
        "@com_google_protobuf//:protobuf",
        "@local_config_cuda//cuda:cudart",
        "@opencv//:imgcodecs",
        "@com_google_absl//:absl",
        "@local_config_cuda//cuda:cuda_headers",
    ]
)

apollo_cc_library(
    name = "apollo_perception_inference_rt_net",
    hdrs = [
        "rt_net.h",
    ],
    srcs = [
        "rt_net.cc",
    ],
    deps = [
        "//cyber",
        ":apollo_perception_inference_rt_common",
        ":apollo_perception_common_inference_tensorrt_plugins",
        "//modules/perception/common/base:apollo_perception_common_base",
        "@local_config_cuda//cuda:cudnn_header",
        "@local_config_tensorrt//:tensorrt",
        "//modules/perception/common/proto:rt_cc_proto",
        "//modules/perception/common/inference:inference_lib",
        "@com_google_protobuf//:protobuf",
        "@local_config_cuda//cuda:cudart",
        "@opencv//:imgcodecs",
        "@com_google_absl//:absl",
        "@local_config_cuda//cuda:cuda_headers",
    ]
)

apollo_cc_test(
    name = "batch_stream_test",
    size = "small",
    srcs = ["batch_stream_test.cc"],
    copts = ["-fno-access-control"],
    data = [
        "//modules/perception/common/inference:inference_test_data",
    ],
    linkstatic = True,
    deps = [
        ":apollo_perception_inference_rt_common",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "entropy_calibrator_test",
    size = "small",
    srcs = ["entropy_calibrator_test.cc"],
    copts = ["-fno-access-control"],
    data = [
        "//modules/perception/common/inference:inference_test_data",
    ],
    linkstatic = True,
    deps = [
        ":apollo_perception_inference_rt_common",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "rt_net_test",
    size = "small",
    srcs = ["rt_net_test.cc"],
    data = [
        "//modules/perception/common/inference:inference_test_data",
    ],
    linkstatic = True,
    tags = ["exclude"],
    deps = [
        ":apollo_perception_inference_rt_net",
        "//modules/perception/common/inference:apollo_perception_common_inference",
        "@atlas//:cblas",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "rt_utils_test",
    size = "small",
    srcs = ["rt_utils_test.cc"],
    linkstatic = True,
    deps = [
        ":apollo_perception_inference_rt_common",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "rt_common_test",
    size = "small",
    srcs = ["rt_common_test.cc"],
    deps = [
        ":apollo_perception_inference_rt_common",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_package()
cpplint()
