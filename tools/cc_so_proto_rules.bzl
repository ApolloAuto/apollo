load("@com_github_grpc_grpc//bazel:generate_cc.bzl", "generate_cc")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")

def cc_so_proto_library(name,
                    srcs = [],
                    deps = [],
                    well_known_protos = False,
                    **kwargs):
  """Generates C++ proto dynamic library from a proto_library.
  Assumes the generated classes will be used in cc_api_version = 2.
  Arguments:
      name: name of rule.
      srcs: a list of C++ proto_library which provides
        the compiled code of any message that the services depend on.
      deps: a list of C++ cc_so_proto_library which provides
        hdrs that the services depend on.
      well_known_protos: Should this library additionally depend on well known
        protos
      **kwargs: rest of arguments, e.g., compatible_with and visibility.
  """
  if len(srcs) > 1:
    fail("Only one srcs value supported", "srcs")

  codegen_target = "_" + name + "_codegen"
  libso_target = "lib" + name + ".so"
  export_hdr_target = name + "_hdrs"

  generate_cc(
      name = codegen_target,
      srcs = srcs,
      well_known_protos = well_known_protos,
      **kwargs
  )

  native.cc_binary(
      name = libso_target,
      srcs = [":" + codegen_target],
      deps = deps +["@com_google_protobuf//:protobuf"],
      linkshared = True,
      linkstatic = True,
      visibility = ["//visibility:public"],
      **kwargs
  )
  native.cc_library(
      name = name,
      srcs = [libso_target],
      hdrs = [":" + codegen_target],
      includes = [":" + codegen_target],
      deps = deps + ["@com_google_protobuf//:protobuf"],
      visibility = ["//visibility:public"],
      **kwargs
  )
  # native.filegroup(
  #   name = export_hdr_target,
  #   srcs = [":" + codegen_target],
  # )

  cc_proto_library(
    name = export_hdr_target,
    deps = srcs,
  )
