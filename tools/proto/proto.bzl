load("//tools:python_rules.bzl", "py_proto_library")
load("@rules_python//python:defs.bzl", "py_library", "py_binary")
load("@rules_proto//proto:defs.bzl", external_proto_library = "proto_library", "ProtoInfo")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("@rules_cc//cc:defs.bzl", "cc_library", "cc_binary", "cc_test")

package_path = "@@REPLACE@@"

def apollo_py_library(**kwargs):
    if "deps" not in kwargs:
        py_library(**dict(kwargs))
    else:
        replaced_deps = []
        for i in kwargs["deps"]:
            replaced_deps.append(_get_real_dep_label(i))
        py_library(**dict(kwargs, deps = replaced_deps))

def apollo_py_binary(**kwargs):
    if "deps" not in kwargs:
        py_binary(**dict(kwargs))
    else:
        replaced_deps = []
        for i in kwargs["deps"]:
            replaced_deps.append(_get_real_dep_label(i))
        py_binary(**dict(kwargs, deps = replaced_deps))

def _depset_to_list(x):
    """Helper function to convert depset to list."""
    iter_list = x.to_list() if type(x) == "depset" else x
    return iter_list

def _apollo_proto_impl(ctx):
    ret_info = []
    for t in ctx.attr.srcs:
        if CcInfo in t:
            ret_info.append(t[CcInfo])
        elif PyInfo in t:
            files = ctx.runfiles(files = _depset_to_list(t.default_runfiles.files))
            ret_info.append(DefaultInfo(runfiles = files))
            ret_info.append(t[InstrumentedFilesInfo])
            # ret_info.append(t[PyCcLinkParamsProvider])
            ret_info.append(t[OutputGroupInfo])
            ret_info.append(t[PyInfo])
        else:
            ret_info.append(t[ProtoInfo])
    return ret_info

_proto_rule = rule(
    attrs = {
        "srcs": attr.label_list(allow_files = True, mandatory=True),
    },
    implementation = _apollo_proto_impl,
)

def _cc_proto_clean_rule_impl(ctx):
    cc_infos = []
    files = []
    for t in ctx.attr.srcs:
        if DefaultInfo in t:
            for item in _depset_to_list(t[DefaultInfo].files):
                if item.extension == 'h' or item.extension == 'cc':
                    files.append(item)
        if CcInfo in t:
            cc_infos.append(t[CcInfo])

    merged_info = cc_common.merge_cc_infos(cc_infos = cc_infos)
    return [
        merged_info,
        DefaultInfo(files=depset(files)),
    ]


_cc_proto_clean_rule = rule(
    attrs = {
        "srcs": attr.label_list(providers = [CcInfo])
    },
    implementation = _cc_proto_clean_rule_impl,
)

def _get_real_dep_label(dep):
    if not package_path.startswith("@@"):
        if dep.startswith(":"):
            return dep
        elif dep.startswith("//{}".format(package_path)):
            replace_prefix = dep.replace("//{}".format(package_path), "")
            if len(replace_prefix) == 0 or replace_prefix[0] == "/" or replace_prefix[0] == ":":
                return dep
            else:
                return "{}{}".format("@apollo_src", dep)
        else:
            return "{}{}".format("@apollo_src", dep)
    return dep

def _replace_dep_label(dep_name, dep_package):
    local_src_prefix = "@apollo_src//"
    return "{}{}:{}".format(local_src_prefix, dep_package, dep_name)

def _to_bin_target(name):
    base_info = name.split(":")
    package_name = native.package_name()
    package_name_list = package_name.split("/")
    label = "".join([i[0] for i in package_name_list])
    base_info[-1] = "lib_%s_%s_bin.so" % (base_info[-1], label)
    return ":".join(base_info)

def _to_wrap_target(name):
    base_info = name.split(":")
    base_info[-1] = "_%s_wrap_lib" % base_info[-1]
    return ":".join(base_info)

def _to_py_target(name):
    base_info = name.split(":")
    base_info[-1] = "%s_py_pb2" % base_info[-1]
    return ":".join(base_info)

def proto_library(tags = [], **kwargs):
    source_env = False

    rule_name = kwargs["name"]
    if not rule_name.endswith("proto"):
        fail("The name of the proto_library instance must end with \"proto\", e.g. \"example_proto\".")
    proto_rule_name = "_%s" % kwargs["name"]
    cc_proto_rule_name = "_%s_cc_proto" % rule_name
    py_proto_rule_name = _to_py_target(rule_name)
    cc_bin_rule_name = _to_bin_target(rule_name)
    cc_lib_rule_name = "_%s_cc_lib" % rule_name
    cc_wrap_rule_name = _to_wrap_target(cc_bin_rule_name)

    rule_info_list = kwargs["name"].split("_")
    rule_suffix = rule_info_list[-1]

    # fake cc and py rule for backward compatibility
    additional_fake_cc_proto_name = "_".join(
        rule_info_list[:len(rule_info_list)-1] + ["cc", "proto"])
    additional_fake_py_proto_name = "_".join(
        rule_info_list[:len(rule_info_list)-1] + ["py", "pb2"])

    if package_path.startswith("@@"):
        # source env
        source_env = True

    external_deps = kwargs["deps"] if "deps" in kwargs else []
    external_cc_deps = []
    external_3rd_proto_deps = []
    for dep in external_deps:
        if not dep.startswith("@"):
            external_cc_deps.append(_get_real_dep_label(dep))
        else:
            external_3rd_proto_deps.append(dep)

    # kwargs["deps"] = replaced_deps
    kwargs["name"] = proto_rule_name

    # depend_libraries = []
    # if source_env:
    #     for d in kwargs["deps"]:
    #         if d.startswith("@"):
    #             continue
    #         depend_libraries.append(d)
    # else:
    #     for d in kwargs["deps"]:
    #         if d.startswith("@") and not d.startswith("@apollo_src"):
    #             continue
    #         depend_libraries.append(d)
    # depend_libraries = kwargs["deps"]

    # origin proto target
    external_proto_library(
        **(dict(kwargs, deps=external_3rd_proto_deps + external_cc_deps)),
    )

    # cc proto target
    cc_proto_library(
        name = cc_proto_rule_name,
        deps = [":%s" % proto_rule_name],
    )

    _cc_proto_clean_rule(
        name = "%s_clean" % (cc_proto_rule_name,),
        srcs = [":%s" % (cc_proto_rule_name,)]
    )

    # py proto target
    py_proto_library(
        name = py_proto_rule_name,
        deps = [":%s" % proto_rule_name] + [_to_py_target(d) for d in external_cc_deps],
    )
    # cc library for linking all symbol
    # cc_library(
    #     name = cc_lib_rule_name,
    #     linkstatic = True,
    #     alwayslink = True,
    #     srcs = [":%s" % cc_proto_rule_name],
    #     # deps = [":%s" % cc_proto_rule_name],# + depend_libraries,
    # )


    native.cc_library(
        name = cc_lib_rule_name,
        linkstatic = True,
        alwayslink = True,
        srcs = [":%s_clean" % (cc_proto_rule_name,)],
        deps = ["@com_google_protobuf//:protobuf"] + external_cc_deps,
        # deps = [":%s" % cc_proto_rule_name],
    )

    native.cc_binary(
        name = cc_bin_rule_name,
        linkshared = True,
        linkstatic = True,
        tags = ["export_library", rule_name],
        # srcs = [":%s_clean" % (cc_proto_rule_name,)],
        # srcs = [":%s" % cc_lib_rule_name],
        deps = ["@com_google_protobuf//:protobuf"] + external_cc_deps + [cc_lib_rule_name],
    )


    # cc_library(
    #     name = cc_wrap_rule_name,
    #     srcs = [":%s" % cc_bin_rule_name] + [cc_lib_rule_name],
    #     deps = depend_libraries + ["@com_google_protobuf//:protobuf"],
    #     hdrs = [":%s" % cc_proto_rule_name],
    #     alwayslink = True,
    #     visibility = ["//visibility:public"],
    # )

    native.cc_library(
        name = cc_wrap_rule_name,
        srcs = [":%s" % cc_bin_rule_name],
        deps = ["@com_google_protobuf//:protobuf"] + external_cc_deps + [":%s_clean" % (cc_proto_rule_name,)],
        # hdrs = [":%s" % cc_proto_rule_name],
        alwayslink = True,
        visibility = ["//visibility:public"],
    )

    # produce single shared library with all symbol
    # cc_binary(
    #     name = cc_bin_rule_name,
    #     srcs = [":%s" % cc_lib_rule_name],
    #     # deps = [":%s" % cc_lib_rule_name] + depend_libraries,
    #     linkshared = True,
    #     linkstatic = True,
    #     # deps = [":%s" % cc_lib_rule_name],
    # )

    # wrapper for return cc_library CcInfo
    # cc_library(
    #     name = cc_wrap_rule_name,
    #     srcs = [":%s" % cc_bin_rule_name],
    #     hdrs = [":%s" % cc_proto_rule_name] + depend_libraries,
    #     deps = depend_libraries + [":%s" % cc_proto_rule_name],
    #     alwayslink = True,
    #     visibility = ["//visibility:public"],
    # )

    _proto_rule(
        name = rule_name,
        srcs = [
            ":%s" % proto_rule_name,
            ":%s" % py_proto_rule_name,
            ":%s" % cc_wrap_rule_name,
        ],
    )

    _proto_rule(
        name = additional_fake_cc_proto_name,
        srcs = [
            ":%s" % proto_rule_name,
            ":%s" % py_proto_rule_name,
            ":%s" % cc_wrap_rule_name,
        ],
    )

    _proto_rule(
        name = additional_fake_py_proto_name,
        srcs = [
            ":%s" % proto_rule_name,
            ":%s" % py_proto_rule_name,
            ":%s" % cc_wrap_rule_name,
        ],
    )
