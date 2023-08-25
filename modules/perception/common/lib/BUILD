load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "apollo_perception_common_lib",
    srcs = [
        "calibration_service/online_calibration_service/online_calibration_service.cc",
        "calibrator/common/histogram_estimator.cc",
        "calibrator/laneline/lane_based_calibrator.cc",
        "calibrator/laneline/laneline_calibrator.cc",
        "config_manager/config_manager.cc",
        "registerer/registerer.cc",
        "thread/thread.cc",
        "thread/thread_pool.cc",
        "thread/thread_worker.cc",
    ],
    hdrs = [
        "calibration_service/online_calibration_service/online_calibration_service.h",
        "calibrator/common/histogram_estimator.h",
        "calibrator/laneline/lane_based_calibrator.h",
        "calibrator/laneline/lane_struct_for_calib.h",
        "calibrator/laneline/laneline_calibrator.h",
        "config_manager/config_manager.h",
        "interface/base_calibration_service.h",
        "interface/base_calibrator.h",
        "interface/base_cipv.h",
        "interface/base_init_options.h",
        "registerer/registerer.h",
        "thread/concurrent_queue.h",
        "thread/mutex.h",
        "thread/thread.h",
        "thread/thread_pool.h",
        "thread/thread_worker.h",
    ],
    deps = [
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/common/algorithm:apollo_perception_common_algorithm",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/camera:apollo_perception_common_camera_camera_frame",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        "//modules/perception/common/proto:perception_config_schema_cc_proto",
        "@eigen",
    ],
)

apollo_cc_test(
    name = "registerer_test",
    size = "small",
    srcs = ["registerer/registerer_test.cc"],
    linkstatic = True,
    deps = [
        ":apollo_perception_common_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "concurrent_queue_test",
    size = "small",
    srcs = ["thread/concurrent_queue_test.cc"],
    linkstatic = True,
    deps = [
        ":apollo_perception_common_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "thread_pool_test",
    size = "small",
    srcs = ["thread/thread_pool_test.cc"],
    deps = [
        ":apollo_perception_common_lib",
        "@com_google_googletest//:gtest_main",
        "@com_google_protobuf//:protobuf",
    ],
)

apollo_cc_test(
    name = "thread_test",
    size = "small",
    srcs = ["thread/thread_test.cc"],
    linkstatic = True,
    deps = [
        ":apollo_perception_common_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "thread_worker_test",
    size = "small",
    srcs = ["thread/thread_worker_test.cc"],
    linkstatic = True,
    deps = [
        ":apollo_perception_common_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_package()
cpplint()