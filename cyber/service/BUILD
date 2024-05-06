load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "cyber_service",
    hdrs = [
        "client.h",
        "service.h",
        "service_base.h",
        "client_base.h"
    ],
    deps = [
        "//cyber/scheduler:cyber_scheduler",
    ],
)

apollo_package()

cpplint()
