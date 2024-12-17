load("//tools:python_rules.bzl", "py_proto_library")
load("@rules_python//python:defs.bzl", "py_library", "py_binary")
load("@rules_proto//proto:defs.bzl", external_proto_library = "proto_library", "ProtoInfo")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("//tools:common.bzl", "select2dict", "list_str2list")
load("@rules_cc//cc:defs.bzl", "cc_library", "cc_binary", "cc_test")

package_path = "@@REPLACE@@"

def apollo_py_library(**kwargs):
    if "deps" not in kwargs:
        py_library(**dict(kwargs))
    else:
        replaced_deps = []
        for i in kwargs["deps"]:
            replaced_deps.append(_get_python_dep_label(i))
        py_library(**dict(kwargs, deps = replaced_deps))

def apollo_py_binary(**kwargs):
    if "deps" not in kwargs:
        py_binary(**dict(kwargs))
    else:
        replaced_deps = []
        for i in kwargs["deps"]:
            replaced_deps.append(_get_python_dep_label(i))
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

def _get_python_dep_label(dep):
    if not package_path.startswith("@@"):
        if dep.startswith(":"):
            return "{}{}{}".format("@apollo_src//", native.package_name(), dep)
        else:
            return "{}{}".format("@apollo_src", dep)
    return dep

def _get_real_dep_label(dep):
    if dep.startswith("@") or "third_party" in dep or (not dep.endswith("_proto") and not dep.endswith("_py_pb2")):
        return dep
    if not package_path.startswith("@@"):
        if dep.startswith(":"):
            return "{}{}{}".format("@apollo_src//", native.package_name(), dep)
        else:
            return "{}{}".format("@apollo_src", dep)
    return dep

def _to_bin_target(name):
    base_info = name.split(":")
    package_name = native.package_name()
    package_name_list = package_name.split("/")
    label = "".join([i[0] for i in package_name_list])
    base_info[-1] = "lib_%s_%s_bin.so" % (base_info[-1], label)
    return ":".join(base_info)

def _to_short_target(name):
    target_name = name.split(":")[-1].replace("_proto", "")
    return target_name

def _to_wrap_target(name):
    base_info = name.split(":")
    base_info[-1] = "_%s_w" % base_info[-1]
    return ":".join(base_info)

def _to_py_target(name):
    base_info = name.split(":")
    base_info[-1] = "%s_py_pb2" % base_info[-1]
    return ":".join(base_info)

def proto_library(tags = [], **kwargs):

    rule_name = kwargs["name"]
    if not rule_name.endswith("proto"):
        fail("The name of the proto_library instance must end with \"proto\", e.g. \"example_proto\".")
    proto_rule_name = "_%s" % kwargs["name"]
    cc_proto_rule_name = "_%s_cc_proto" % rule_name
    py_proto_rule_name = _to_py_target(rule_name)
    cc_bin_rule_name = _to_bin_target(rule_name)
    cc_lib_rule_name = "_%s_cc_lib" % rule_name
    cc_wrap_rule_name = _to_wrap_target(_to_short_target(rule_name))

    rule_info_list = kwargs["name"].split("_")
    rule_suffix = rule_info_list[-1]

    # fake cc and py rule for backward compatibility
    additional_fake_cc_proto_name = "_".join(
        rule_info_list[:len(rule_info_list)-1] + ["cc", "proto"])
    additional_fake_py_proto_name = "_".join(
        rule_info_list[:len(rule_info_list)-1] + ["py", "pb2"])

    external_deps = kwargs["deps"] if "deps" in kwargs else []
    external_cc_deps = []
    external_3rd_proto_deps = []
    for dep in external_deps:
        if not dep.startswith("@"):
            external_cc_deps.append(_get_real_dep_label(dep))
        else:
            external_3rd_proto_deps.append(dep)

    kwargs["name"] = proto_rule_name

    # origin proto target
    external_proto_library(
        **(dict(kwargs, deps=external_3rd_proto_deps + external_cc_deps, tags = ["exclude"],)),
    )

    # cc proto target
    cc_proto_library(
        name = cc_proto_rule_name,
        deps = [":%s" % proto_rule_name],
        tags = ["exclude"],
    )

    _cc_proto_clean_rule(
        name = "%s_clean" % (cc_proto_rule_name,),
        srcs = [":%s" % (cc_proto_rule_name,)],
        tags = ["exclude"],
    )

    # py proto target
    py_proto_library(
        name = py_proto_rule_name,
        deps = [":%s" % proto_rule_name] + [_to_py_target(d) for d in external_cc_deps],
        tags = ["exclude"],
    )

    native.cc_library(
        name = cc_lib_rule_name,
        linkstatic = True,
        alwayslink = True,
        srcs = ["@apollo_src//%s:%s_clean" % (native.package_name(), cc_proto_rule_name,)], 
        deps = ["@com_google_protobuf//:protobuf"] + external_cc_deps,
        tags = ["exclude"],
    )

    native.cc_binary(
        name = cc_bin_rule_name,
        linkshared = True,
        linkstatic = True,
        tags = ["export_library", rule_name, "exclude"],
        deps = ["@com_google_protobuf//:protobuf"] + external_cc_deps + ["@apollo_src//%s:%s" % (native.package_name(), cc_lib_rule_name)],
    )

    native.cc_library(
        name = cc_wrap_rule_name,
        srcs = [":%s" % cc_bin_rule_name],
        deps = ["@com_google_protobuf//:protobuf"] + external_cc_deps + ["@apollo_src//%s:%s_clean" % (native.package_name(), cc_proto_rule_name,)],
        alwayslink = True,
        tags = ["exclude"],
        visibility = ["//visibility:public"],
    )

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

def _filter_proto_impl(ctx):
    cc_infos = []
    index = 0
    for dep in ctx.attr.deps:
        if CcInfo in dep and PyInfo in dep and ProtoInfo in dep:
            cc_infos.append(ctx.attr.apollo_src_deps[index][CcInfo])
        else:
            cc_infos.append(dep[CcInfo])
        index = index + 1
    merged_info = cc_common.merge_cc_infos(cc_infos = cc_infos)
    return [
        DefaultInfo(),
        merged_info
    ]

filter_proto_library = rule(
    implementation = _filter_proto_impl,
    attrs = {
        "deps": attr.label_list(providers = [CcInfo]),
        "apollo_src_deps": attr.label_list(providers = [CcInfo]),
    },
)

def parse_select(deps):
    parsed_deps = []
    apollo_src_deps = []
    select_dict_list = []
    temp = {}
    if type(deps) == "select":
        deps_list = []
        for group_str in str(deps).strip().split(" + "):
            if "select({" in group_str.strip():
                select_dict_list.append(select2dict(group_str))
            elif group_str.strip() == "[]":
                continue
            else:
                deps_list += list_str2list(group_str.strip())
        parsed_deps = deps_list

        for s in parsed_deps:
            temp[s] = s
        parsed_deps = [s for s in temp]
        apollo_src_deps = [_get_real_dep_label(s) for s in temp]

        if len(select_dict_list) != 0:
            for i in select_dict_list:
                parsed_deps += select(i)
                apollo_src_deps += select(i)
    else:
        parsed_deps = deps
        apollo_src_deps = [_get_real_dep_label(s) for s in deps]

    return parsed_deps, apollo_src_deps

def fix_proto_cc_binary_wrap(name, deps, **kwargs):
    parsed_deps, apollo_src_deps = parse_select(deps)
    filter_proto_library(name=name + "_filter", deps=parsed_deps, apollo_src_deps=apollo_src_deps)
    native.cc_binary(**dict(kwargs, name=name, deps=[name + "_filter", "@apollo_src//:hdrs"]))

def fix_proto_cc_library_wrap(name, deps, **kwargs):
    parsed_deps, apollo_src_deps = parse_select(deps)
    filter_proto_library(name=name + "_filter", deps=parsed_deps, apollo_src_deps=apollo_src_deps)
    native.cc_library(**dict(kwargs, name=name, deps=[name + "_filter", "@apollo_src//:hdrs"]))

def fix_proto_cc_test_wrap(name, deps, **kwargs):
    parsed_deps, apollo_src_deps = parse_select(deps)
    filter_proto_library(name=name + "_filter", deps=parsed_deps, apollo_src_deps=apollo_src_deps)
    native.cc_test(**dict(kwargs, name=name, deps=[name + "_filter", "@apollo_src//:hdrs"]))
