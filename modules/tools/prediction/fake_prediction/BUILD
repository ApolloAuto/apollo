load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libfake_prediction_component.so",
    srcs = ["fake_prediction.cc"],
    linkshared = True,
    linkstatic = False,
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:message_util",
        "//modules/prediction/proto:prediction_proto",
    ],
)

cpplint()
