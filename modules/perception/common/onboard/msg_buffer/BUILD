load("@rules_cc//cc:defs.bzl", "cc_library", "cc_binary")
load("//tools:apollo_package.bzl", "apollo_package")

# DO NOT MODIFY
# THIS FILE IS GENERATED AUTOMATICALLY TO HELP WRITE BUILD FILE
# DELETE THIS FILE WHEN YOU HAVE CREATED THE COMPLETE BUILD FILE
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libmsg_buffer.so",
    srcs = ["msg_buffer.cc", "msg_buffer.h"],
    linkstatic = True,
    linkshared = True,
    deps = [
        "//cyber",
        "@boost",
	"@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "msg_buffer",
    srcs = [":libmsg_buffer.so"],
    hdrs = ["msg_buffer.h"],
    deps = [
        "//cyber",
        "@boost",
        "@com_github_gflags_gflags//:gflags",
    ],
)


apollo_package()
cpplint()
