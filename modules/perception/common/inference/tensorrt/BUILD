load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_cc_test", "apollo_package")
load("//third_party/gpus:common.bzl", "gpu_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

gpu_library(
    name = "apollo_perception_common_inference_tensorrt_plugins",
    srcs = [
        "plugins/argmax_plugin.cu",
        "plugins/dfmb_psroi_align_plugin.cu",
        "plugins/kernels.cu",
        "plugins/leakyReLU_plugin.cu",
        "plugins/nms_cuda.cu",
        "plugins/rcnn_proposal_plugin.cu",
        "plugins/rpn_proposal_ssd_plugin.cu",
        "plugins/slice_plugin.cu",
        "plugins/softmax_plugin.cu",
    ],
    hdrs = [
        "plugins/argmax_plugin.h",
        "plugins/dfmb_psroi_align_plugin.h",
        "plugins/kernels.h",
        "plugins/leakyReLU_plugin.h",
        "plugins/rcnn_proposal_plugin.h",
        "plugins/rpn_proposal_ssd_plugin.h",
        "plugins/slice_plugin.h",
        "plugins/softmax_plugin.h",
    ],
    deps = [
        ":apollo_perception_inference_rt_common",
        "//cyber",
        "//modules/perception/common/base:apollo_perception_common_base",
        "@eigen",
        "@local_config_cuda//cuda:cublas",
        "@local_config_cuda//cuda:cudart",
        "@local_config_cuda//cuda:cudnn",
        "@local_config_tensorrt//:tensorrt",
    ],
    alwayslink = True,
)

apollo_cc_test(
    name = "slice_plugin_test",
    size = "small",
    srcs = ["plugins/slice_plugin_test.cc"],
    linkstatic = True,
    deps = [
        ":apollo_perception_common_inference_tensorrt_plugins",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "argmax_plugin_test",
    size = "small",
    srcs = ["plugins/argmax_plugin_test.cc"],
    linkstatic = True,
    deps = [
        ":apollo_perception_common_inference_tensorrt_plugins",
        "@com_google_googletest//:gtest_main",
        "@local_config_cuda//cuda:cublas",
    ],
)

apollo_cc_library(
    name = "apollo_perception_inference_rt_common",
    srcs = [
        "batch_stream.cc",
        "entropy_calibrator.cc",
        "rt_common.cc",
        "rt_utils.cc",
    ],
    hdrs = [
        "batch_stream.h",
        "entropy_calibrator.h",
        "rt_common.h",
        "rt_legacy.h",
        "rt_utils.h",
    ],
    deps = [
        "//cyber",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/inference:inference_lib",
        "//modules/perception/common/proto:rt_cc_proto",
        "@com_google_absl//:absl",
        "@com_google_protobuf//:protobuf",
        "@local_config_cuda//cuda:cuda_headers",
        "@local_config_cuda//cuda:cudart",
        "@local_config_cuda//cuda:cudnn_header",
        "@local_config_tensorrt//:tensorrt",
        "@opencv//:imgcodecs",
    ],
)

apollo_cc_library(
    name = "apollo_perception_inference_rt_net",
    srcs = [
        "rt_net.cc",
    ],
    hdrs = [
        "rt_net.h",
    ],
    deps = [
        ":apollo_perception_common_inference_tensorrt_plugins",
        ":apollo_perception_inference_rt_common",
        "//cyber",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/inference:inference_lib",
        "//modules/perception/common/proto:rt_cc_proto",
        "@com_google_absl//:absl",
        "@com_google_protobuf//:protobuf",
        "@local_config_cuda//cuda:cuda_headers",
        "@local_config_cuda//cuda:cudart",
        "@local_config_cuda//cuda:cudnn_header",
        "@local_config_tensorrt//:tensorrt",
        "@opencv//:imgcodecs",
    ],
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
