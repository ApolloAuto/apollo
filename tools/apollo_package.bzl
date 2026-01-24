load("//tools/install:install.bzl", "install", "install_files", "install_src_files", "install_plugin")
load("//tools:apollo.bzl", "cyber_plugin_description")
load("//tools/package:dynamic_deps.bzl", "STATUS", "SOURCE", "BINARY")
load("@rules_cc//cc:defs.bzl", legacy_cc_library = "cc_library", legacy_cc_binary = "cc_binary", legacy_cc_test = "cc_test")


INSATLL_LABEL_NAME = "install"
INSTALL_SRC_LABEL_NAME = "install_src"

SHARED_LIB_OR_BIN_RULE = "cc_binary"
DEFAULT_LIB_RULE = "cc_library"
INSTALL_RULE = "_install_rule"
INSTALL_FILE_RULE = "_install_files_rule"
INSTALL_SRC_RULE = "_install_src_files_rule"
DATA_RULE = "filegroup"
PYTHON_LIB_RULE = "py_library"
PAYTHON_BIN_RULE = "py_binary"
PROTO_RULE = "proto_library"
CPP_PROTO_RULE = "cc_proto_library"
CPP_TEST_RULE = "cc_test"

PLUGIN_RULE = "cyber_plugin_description"

APOLLO_COMPONENT_LIBRARY_PREFIX = "DO_NOT_IMPORT_"

CC_LIBRARY = native.cc_library if STATUS == 2 else legacy_cc_library
CC_BINARY = native.cc_binary if STATUS == 2 else legacy_cc_binary
CC_TEST = native.cc_test if STATUS == 2 else legacy_cc_test

def _select2dict(select_str):
    result = dict()
    cxt_str = select_str[:-2].replace("select({","")
    for kv_str in cxt_str.split("],"):
      k_str, v_str = kv_str.strip().split(": [")
      if "" == v_str.strip() or "]" == v_str.strip():
        result[k_str.strip()[1:-1]] = []
      else:
        v_list = []
        v_cxt = v_str.strip()
        if v_cxt[-1] == "]":
          v_cxt = v_str.strip()[:-1]
        for v_v in v_cxt.split(","):
          v_list.append(v_v.strip()[1:-1])
        result[k_str.strip()[1:-1]] = v_list

    return result

def _list_str2list(list_str):
    result = []
    cxt = list_str[1:-1]
    for l_str in cxt.strip().split(","):
      result.append(l_str.strip()[1:-1])
    return result

def _is_lib(name):
    if name.startswith("lib") and name.endswith(".so"):
        return True
    elif name.endswith(".so"):
        return True
    return False

def _is_plugin_package():
    for rule in native.existing_rules().values():
        if rule["kind"] == PLUGIN_RULE:
            return True
    return False 

def _is_plugin_label(rule_instance):
    for rule in native.existing_rules().values():
        if rule["kind"] == PLUGIN_RULE and rule["plugin"].endswith(rule_instance["name"]):
            return True
    return False 

def _find_description(rule_instance):
    for rule in native.existing_rules().values():
        if rule["kind"] == PLUGIN_RULE and rule["plugin"].endswith(rule_instance["name"]):
            return rule["description"]
    return None

def _need_autoconf_install():
    has_install_rule = None
    has_install_src_rule = None
    for rule in native.existing_rules().values():
        if rule["name"] == INSATLL_LABEL_NAME and rule["kind"] == INSTALL_RULE:
            has_install_rule = True
        if rule["name"] == INSTALL_SRC_LABEL_NAME and rule["kind"] == INSTALL_SRC_RULE:
            has_install_src_rule = True
    if has_install_rule == None:
        has_install_rule = False
    if has_install_src_rule == None:
        has_install_src_rule = False
    return has_install_rule, has_install_src_rule

def _add_install_rules(install_actions, install_src_actions, 
                        has_install_rule, has_install_src_rule, 
                        package_name,subpackages_install_target, 
                        subpackages_install_src_target):
    if not has_install_rule:
        for action in install_actions:
            if action["kind"] == SHARED_LIB_OR_BIN_RULE:
                if action["label"] == "lib":
                    if "plguin" in action and action["plguin"]:
                        install_plugin(
                            name = action["name"],
                            description = action["description"],
                            plugin = action["targets"][0],
                            visibility = ["//visibility:public"],
                        )
                    else:
                        rule_tags = action["tags"] if "tags" in action else []
                        install(
                            name = action["name"],
                            targets = action["targets"],
                            library_dest = action["library_dest"],
                            type = action["type"],
                            tags = rule_tags,
                            package_path = package_name, 
                            visibility = ["//visibility:public"],
                        )
                else:
                    install(
                        name = action["name"],
                        targets = action["targets"],
                        runtime_dest = action["runtime_dest"],
                        type = action["type"],
                        package_path = package_name, 
                        visibility = ["//visibility:public"],
                    )
            elif action["kind"] == DATA_RULE:
                install(
                    name = action["name"],
                    data = action["data"],
                    data_dest = action["data_dest"],
                    type = action["type"],
                    package_path = package_name,
                    visibility = ["//visibility:public"],
                ) 
            elif action["kind"] == PYTHON_LIB_RULE:
                install_files(
                    name = action["name"],
                    files = action["files"],
                    dest = action["dest"],
                    type = action["type"],
                    package_path = package_name,
                    visibility = ["//visibility:public"],
                )
            elif action["kind"] == PAYTHON_BIN_RULE:
                install(
                    name = action["name"],
                    targets = action["targets"],
                    py_dest = action["py_dest"],
                    type = action["type"],
                    package_path = package_name,
                    visibility = ["//visibility:public"],
                )
            elif action["kind"] == CPP_PROTO_RULE:
                install(
                    name = action["name"],
                    data = action["data"],
                    data_dest = action["data_dest"],
                    type = action["type"],
                    package_path = package_name,
                    visibility = ["//visibility:public"],
                )
            elif action["kind"] == DEFAULT_LIB_RULE:
                install(
                    name = action["name"],
                    data = action["data"],
                    data_dest = action["data_dest"],
                    type = action["type"],
                    package_path = package_name, 
                    visibility = ["//visibility:public"], 
                )
            else:
                continue

        install(
            name = "install",
            deps = [i["name"] for i in install_actions] + subpackages_install_target,
            type = "neo",
            package_path = package_name,
            visibility = ["//visibility:public"],
        )
    if not has_install_src_rule:
        for action in install_src_actions:
            if action["type"] != "disable_source":
                install_src_files(
                    name = action["name"],
                    src_dir =  action["src_dir"],
                    dest = action["dest"],
                    filter = action["filter"],
                    type = action["type"],
                    visibility = ["//visibility:public"],
                )
            else:
                install_src_files(
                    name = action["name"],
                    src_dir =  action["src_dir"],
                    dest = action["dest"],
                    filter = action["filter"],
                    type = action["type"],
                    visibility = ["//visibility:public"],
                ) 
        
        install_src_files(
            name = "install_src",
            deps = [i["name"] for i in install_src_actions] + subpackages_install_src_target,
            type = "neo",
            visibility = ["//visibility:public"],
        )

def _generate_cyberfile_rule(package_install_target, package_name):
    install_action_instance = {}
    install_action_instance["kind"] = DATA_RULE
    install_action_instance["name"] = "_install_cyberfile"
    install_action_instance["type"] = "neo"
    install_action_instance["data"] = native.glob(["cyberfile.xml"])
    install_action_instance["data_dest"] = package_name
    package_install_target.append(install_action_instance)

def _generate_plugin_rule(package_install_target, package_name):
    for rule in native.existing_rules().values():
        if rule["kind"] != PLUGIN_RULE:
            continue
        install_action_instance = {}
        install_action_instance["kind"] = DATA_RULE
        install_action_instance["name"] = "_install_%s" % rule["description"].replace(":", "")
        install_action_instance["type"] = "neo"
        install_action_instance["data"] = [rule["description"]]
        install_action_instance["data_dest"] = "plugin_meta@%s@%s" % (
            package_name, rule["description"].replace(":", ""))
        package_install_target.append(install_action_instance)

def apollo_package(enable_source=True):
    """For every rule which should be installed in the BUILD file so far, 
    adds a install rule that runs installation over the sources listed in that rule.  
    Thus, BUILD file authors should call this function at the *end* of every BUILD file.
    """
    has_install_rule, has_install_src_rule = _need_autoconf_install()
    if has_install_rule and has_install_src_rule:
        return 

    package_install_target = []
    package_name = native.package_name()

    subpackages = native.subpackages(include=["**/*"], allow_empty=True)
    subpackages_install_target = [
        "//%s/%s:%s" % (package_name, i, "install") for i in subpackages
    ]
    subpackages_install_src_target = [
        "//%s/%s:%s" % (package_name, i, "install_src") for i in subpackages
    ]
    
    for rule in native.existing_rules().values():
        if "testonly" in rule and rule["testonly"] == True:
            continue
        install_action_instance = {}
        install_action_instance["kind"] = rule["kind"]
        install_action_instance["name"] = "_install_%s" % rule["name"]
        if rule["kind"] == SHARED_LIB_OR_BIN_RULE:
            install_action_instance["targets"] = [":%s" % rule["name"]]
            if _is_lib(rule["name"]):
                install_action_instance["type"] = "neo"
                install_action_instance["label"] = "lib"
                install_action_instance["library_dest"] = "lib/%s" % package_name 
                if "tags" in rule and len(rule["tags"]) >= 2 and "export_library" in rule["tags"]:
                    rule_tags = [rule["tags"][len(rule["tags"])-2], rule["tags"][-1]]
                    install_action_instance["tags"] = rule["tags"]
                if _is_plugin_label(rule):
                    install_action_instance["plguin"] = True
                    install_action_instance["description"] = _find_description(rule)
            else:
                install_action_instance["type"] = "neo"
                install_action_instance["label"] = "bin"
                install_action_instance["runtime_dest"] = "bin"
            package_install_target.append(install_action_instance)
        elif rule["kind"] == DATA_RULE:
            install_action_instance["type"] = "neo"
            install_action_instance["data"] = [":%s" % rule["name"]]
            install_action_instance["data_dest"] = "share/%s" % package_name
            package_install_target.append(install_action_instance) 
        elif rule["kind"] == PYTHON_LIB_RULE:
            install_action_instance["type"] = "neo"
            install_action_instance["files"] = [":%s" % rule["name"]]
            install_action_instance["dest"] = "python/%s" % package_name
            package_install_target.append(install_action_instance)
        elif rule["kind"] == PAYTHON_BIN_RULE:
            install_action_instance["type"] = "neo"
            install_action_instance["targets"] = [":%s" % rule["name"]]
            install_action_instance["py_dest"] = "bin"
            package_install_target.append(install_action_instance)
        elif rule["kind"] == CPP_PROTO_RULE:
            install_action_instance["type"] = "neo"
            install_action_instance["data"] = [":%s" % rule["name"]]
            install_action_instance["data_dest"] = "include/%s" % package_name 
            package_install_target.append(install_action_instance)
        elif rule["kind"] == DEFAULT_LIB_RULE:
            if "tags" in rule and "shared_library" in rule["tags"]:
                install_action_instance["type"] = "neo"
                install_action_instance["data"] = rule["srcs"]
                install_action_instance["data_dest"] = "lib/%s" % package_name
                package_install_target.append(install_action_instance)
        else:
            continue
    
    _generate_cyberfile_rule(package_install_target, package_name)
    if _is_plugin_package():
        _generate_plugin_rule(package_install_target, package_name)
    
    if enable_source:
        src_code_action = {
            "name": "install_module_src",
            "src_dir": ["."],
            "dest": "src/%s" % package_name,
            "filter": "*",
            "type": "neo"
        }
    else:
        src_code_action = {
            "name": "install_module_src",
            "src_dir": ["__DO_NOT_INSTALL__"],
            "dest": "src/%s" % package_name,
            "filter": "*",
            "type": "disable_source"
        } 
    header_action = {
        "name": "install_header_src",
        "src_dir": ["."],
        "dest": "include/%s" % package_name,
        "filter": "*.h*",
        "type": "neo",
    }

    _add_install_rules(
        package_install_target, [src_code_action, header_action],
        has_install_rule, has_install_src_rule, package_name,
        subpackages_install_target, subpackages_install_src_target)

def _replace_result(origin_target, pkg_name):
    if ":" not in origin_target:
        origin_target = "{}:{}".format(origin_target, origin_target.split("/")[-1])

    origin_target_split = origin_target.split(":")
    target_name = origin_target_split[-1] if "proto" not in origin_target_split[-1] else origin_target_split[-1].replace("_cc_", "_")
    prefix = origin_target_split[0].replace("//", "")
    # Consistent with the way target is converted in the meta information: tools/install/install.py.in
    replaced_target_name = "{}_C{}".format(prefix.replace("/", "_S"), target_name)
    # if len(replaced_target_name) > 100:
    #     replaced_target_name = replaced_target_name[int(len(replaced_target_name) / 2):]

    replaced_repo = BINARY[pkg_name]["targets"][0].split("//:")[0]
    res = "{}//:{}".format(replaced_repo, replaced_target_name)
    if res not in BINARY[pkg_name]["targets"]:
        fail("target {} is invalid since package {} does not have it".format(origin_target, pkg_name))
    return res

def _is_belong_path(instance, path):
    if instance == path:
        return True
    elif instance.startswith(path) and (instance[len(path)] == "/" or instance[len(path)] == ":"):
        return True
    else:
        return False

def _is_replace_instance(instance):
    for pkg in SOURCE:
        if not instance.startswith(SOURCE[pkg]["path"]):
            continue
        if _is_belong_path(instance, SOURCE[pkg]["path"]):
            return False, None
                
    for pkg in BINARY:
        if not instance.startswith(BINARY[pkg]["path"]):
            continue
        if instance == BINARY[pkg]["path"]:
            return True, pkg
        else:
            if instance[len(BINARY[pkg]["path"])] == "/" or instance[len(BINARY[pkg]["path"])] == ":":
                return True, pkg
    
    return False, None

def _replace_deps(deps_list):
    for i in range(len(deps_list)):
        if "@" not in deps_list[i]:
            continue
        else:
            if ":" in deps_list[i]:
                continue
            else:
                lib_name = deps_list[i].replace("//", "").replace("@", "")
                deps_list[i] = deps_list[i] + "//:" + lib_name

    for i in range(len(deps_list)):
        status, pkg_name = _is_replace_instance(deps_list[i])
        if status:
            deps_list[i] = _replace_result(deps_list[i], pkg_name)
    
    new_deps = [] + deps_list

    deps_map = {}
    for i in new_deps:
        deps_map[i] = i
    new_deps = [i for i in deps_map]

    return new_deps    

def _auto_padding_deps(registered_deps, auto_deps=False):
    target_location = native.package_name()
    target_location_with_prefix = "//{}".format(target_location)
    source_pkg = None
    for i in SOURCE:
        if target_location_with_prefix.startswith(SOURCE[i]["path"]) and \
            (len(target_location_with_prefix) == len(SOURCE[i]["path"]) or \
                ("//{}".format(target_location))[len(SOURCE[i]["path"])] == "/" or \
                ("//{}".format(target_location))[len(SOURCE[i]["path"])] == ":"):
            source_pkg = i
            break
    if source_pkg == None:
        ret_deps = []
        for i in registered_deps:
            if (i.startswith("@") and "_C" not in i) or i.startswith(":"):
                continue
            dep_path = (i.split("//:")[1].replace("_S", "/")).split("_C")[0]
            depend_binary = None
            for b in BINARY:
                if "//{}".format(dep_path).startswith(BINARY[b]["path"]) and \
                    (len("//{}".format(dep_path)) == len(BINARY[b]["path"]) or \
                        ("//{}".format(dep_path))[len(BINARY[b]["path"])] == "/" or \
                        ("//{}".format(dep_path))[len(BINARY[b]["path"])] == ":"): 
                    depend_binary = b
                    break
            if depend_binary == None:
                continue
            if len(BINARY[b]["targets"]) > 0:
                hdr_target = "@{}//:{}".format(b, b)
                if hdr_target in BINARY[b]["targets"]:
                    if hdr_target in registered_deps:
                        continue
                    if hdr_target in ret_deps:
                        continue
                    ret_deps.append(hdr_target)
                else:
                    for i in BINARY[b]["targets"]:
                        if i in registered_deps:
                            continue
                        if i in ret_deps:
                            continue
                        ret_deps.append(i)
        for b in BINARY:
            if b.startswith("3rd"):
                for i in BINARY[b]["targets"]:
                    if i not in ret_deps:
                        ret_deps.append(i)
        return ret_deps
        # fail("Can't find package located in {} in SOURCE dict of dynamic-import file".format(target_location))
    ret_deps = []
    for dep in SOURCE[source_pkg]["depends"]:
        if dep not in BINARY:
            fail("Can't find package {} in BINARY dict of dynamic-import file".format(dep))
        if auto_deps:
            for depend_target in BINARY[dep]["targets"]:
                if depend_target in registered_deps:
                    continue
                if APOLLO_COMPONENT_LIBRARY_PREFIX in depend_target:
                    continue
                ret_deps.append(depend_target)
        else:
            if len(BINARY[dep]["targets"]) > 0:
                if dep[0].isdigit():
                    for i in BINARY[dep]["targets"]:
                        if i in registered_deps:
                            continue
                        ret_deps.append(i) 
                else:
                    hdr_target = "@{}//:{}".format(dep, dep)
                    if hdr_target in BINARY[dep]["targets"]:
                        if hdr_target in registered_deps:
                            continue
                        ret_deps.append(hdr_target)
                    else:
                        for i in BINARY[dep]["targets"]:
                            if i in registered_deps:
                                continue
                            ret_deps.append(i)
    return ret_deps

def dynamic_fill_deps(attrs):
    if STATUS != 2:
        attrs.pop("auto_find_deps", None)
        return attrs

    if "deps" not in attrs:
        attrs["deps"] = []

    deps = attrs["deps"]
    
    current_deps_dict = {}
    ret_deps_list = []
    if type(deps) == "select":
        for group_str in str(deps).strip().split(" + "):
            if "select({" in group_str.strip():
                s_deps_dict = _select2dict(group_str.strip())
                for k, v in s_deps_dict.items():
                    if type(v) == "list":
                        s_deps_dict[k] = _replace_deps(v)

                        for i in s_deps_dict[k]:
                            current_deps_dict[i] = i

                ret_deps_list.append(select(s_deps_dict))
            else:
                l_deps_str = group_str.strip()
                l_deps_list = _list_str2list(l_deps_str)
                ret_deps = _replace_deps(l_deps_list)

                for dep in ret_deps:
                    current_deps_dict[dep] = dep

                ret_deps_list.append(ret_deps)

        if "auto_find_deps" in attrs and attrs["auto_find_deps"] == True:
            ret_deps_list.append(_auto_padding_deps(current_deps_dict), True)
        else:
            ret_deps_list.append(_auto_padding_deps(current_deps_dict)) 

        n_deps = []
        for d in ret_deps_list:
            n_deps += d

        attrs["deps"] = n_deps

        attrs.pop("auto_find_deps", None)
        return attrs
    elif type(deps) == "list":
        replaced_deps = _replace_deps(deps)
        for dep in replaced_deps:
            current_deps_dict[dep] = dep
        if "auto_find_deps" in attrs and attrs["auto_find_deps"] == True:
            ws_deps = _auto_padding_deps(current_deps_dict, True)
        else:
            ws_deps = _auto_padding_deps(current_deps_dict)
        n_deps = replaced_deps + ws_deps

        dup = {}
        for i in n_deps:
            dup[i] = i
        n_deps = [i for i in dup]

        attrs["deps"] = n_deps

        attrs.pop("auto_find_deps", None)
        return attrs
    else:
        attrs.pop("auto_find_deps", None)
        return attrs

def apollo_deps_library(**kwargs):
    # for binary package using
    if "deps" not in kwargs:
        # legacy package, skip
        native.cc_library(**dict(kwargs))
    else:
        # online package, redirect the deps to workspace if source existed
        deps = []
        for dep in kwargs["deps"]:
            dep_pkg_name = dep.split("//:")[0].replace("@", "")
            target_with_path = dep.split("//:")[1]
            if dep_pkg_name in SOURCE:
                dep = target_with_path.replace("_S", "/").replace("_C", ":")
                dep = "@//{}".format(dep)
            deps.append(dep)
        native.cc_library(**dict(kwargs, deps = deps))

def apollo_cc_test(**kwargs):
    # simple wrap for cc_test
    CC_TEST(**(dynamic_fill_deps(kwargs)))

def apollo_cc_binary(**kwargs):
    # simple wrap for cc_binary
    CC_BINARY(**(dynamic_fill_deps(kwargs)))

def apollo_component(**kwargs):
    if not kwargs["name"].startswith("lib") or not kwargs["name"].endswith(".so"):
        fail("name must start with 'lib' and end with '.so'")
    if "alwayslink" in kwargs:
        fail("'apollo_component' macro has not 'alwayslink' attribute")

    internal_lib_name = "{}{}".format(
        APOLLO_COMPONENT_LIBRARY_PREFIX, kwargs["name"][3: len(kwargs["name"])-3])

    apollo_cc_library(**dict(dynamic_fill_deps(kwargs), name = internal_lib_name, 
                            visibility = ["//visibility:public"]))
    
    CC_BINARY(
        name = kwargs["name"],
        linkshared = True,
        linkstatic = True,
        deps = [":{}".format(internal_lib_name)],
    )

def apollo_cc_library(**kwargs):
    merge_src = []
    bin_kwargs = {}
    for i in kwargs:
        if i == "hdrs":
            continue
        elif i.endswith("include_prefix"):
            continue
        bin_kwargs[i] = kwargs[i]
    if "srcs" in kwargs:
        merge_src += kwargs["srcs"]
    if "hdrs" in kwargs:
        merge_src += kwargs["hdrs"]
    temp = {}
    
    select_dict_list = []
    if type(merge_src) == "select":
        merge_src_list = []
        for group_str in str(merge_src).strip().split(" + "):
            if "select({" in group_str.strip():
                select_dict_list.append(_select2dict(group_str))
            elif group_str.strip() == "[]":
                continue
            else:
                merge_src_list += _list_str2list(group_str.strip())
        merge_src = merge_src_list

    for s in merge_src:
        temp[s] = s
    merge_src = [s for s in temp]

    if len(select_dict_list) != 0:
        for i in select_dict_list:
            merge_src += select(i)
    
    if "alwayslink" in bin_kwargs:
        bin_kwargs.pop("alwayslink")
    CC_BINARY(**dict(
        dynamic_fill_deps(bin_kwargs), name = "lib{}.so".format(bin_kwargs["name"]),
        linkshared = True, linkstatic = True, srcs = merge_src,
        visibility = ["//visibility:public"], tags = ["export_library", kwargs["name"]]))
    CC_LIBRARY(**dict(
        dynamic_fill_deps(kwargs), srcs = [":lib{}.so".format(kwargs["name"])],
        alwayslink = True, visibility = ["//visibility:public"]))

def apollo_plugin(**kwargs):
    if "description" not in kwargs:
        fail("missing attribution of 'description'")
    if "name" not in kwargs:
        fail("missing attribution of 'name'")
    if not kwargs["name"].startswith("lib") or not kwargs["name"].endswith(".so"):
        fail("name must start with 'lib' and end with '.so'")
    cc_library_name = "{}_lib".format(kwargs["name"][3: len(kwargs["name"])-3])

    cyber_plugin_description(
        name = "plugin_{}_description".format(cc_library_name),
        description = kwargs["description"],
        plugin = kwargs["name"],
    )

    kwargs = dynamic_fill_deps(kwargs)

    CC_LIBRARY(
        name = cc_library_name,
        srcs = kwargs["srcs"] if "srcs" in kwargs else [],
        hdrs = kwargs["hdrs"] if "hdrs" in kwargs else [],
        deps = kwargs["deps"] if "deps" in kwargs else [],
        copts = kwargs["copts"] if "copts" in kwargs else [], 
        alwayslink = True,
    )

    CC_BINARY(
        name = kwargs["name"],
        linkshared = True,
        linkstatic = True,
        deps = [":{}".format(cc_library_name)],
    )

def _file_name(filePathName):
    if "/" in filePathName:
        return filePathName.rsplit("/", -1)[1]
    else:
        return filePathName

def _base_name(fileName):
    return fileName.split(".")[0]

def apollo_qt_library(name, srcs, hdrs, data = [], copts = [], uis = [], res = [], normal_hdrs = [], deps = None, **kwargs):
    warp_kwargs = {"deps": deps}
    deps = dynamic_fill_deps(warp_kwargs)["deps"]
    library_srcs = []
    library_hdrs = []
    for hItem in hdrs:
        base_name = _base_name(_file_name(hItem))
        cmd = """
        if grep -q Q_OBJECT $(location %s); then \
            /usr/local/qt5/bin/moc $(location %s) -o $@ -f'%s'; \
        else \
            echo '' > $@ ; \
        fi""" % (hItem, hItem, "%s/%s" % (native.package_name(), hItem))
        native.genrule(
            name = "%s_moc" % base_name,
            srcs = [hItem],
            outs = ["moc_%s.cpp" % base_name],
            cmd = cmd,
        )
        library_srcs.append("moc_%s.cpp" % base_name)

    for uitem in uis:
        base_name = _base_name(_file_name(uitem))
        native.genrule(
            name = "%s_ui" % base_name,
            srcs = [uitem],
            outs = ["ui_%s.h" % base_name],
            cmd = "/usr/local/qt5/bin/uic $(locations %s) -o $@" % uitem,
        )
        library_hdrs.append("ui_%s.h" % base_name)

    CC_LIBRARY(
        name = "__apollo_interna_qt_deps",
        deps = deps,
    )

    for ritem in res:
        base_name = _base_name(_file_name(ritem))
        native.genrule(
            name = "%s_res" % base_name,
            srcs = [ritem] + [":__apollo_interna_qt_deps"] + data,
            outs = ["res_%s.cpp" % base_name],
            cmd = "/usr/local/qt5/bin/rcc --name res --output $(OUTS) $(location %s)" % ritem,
        )
        library_srcs.append("res_%s.cpp" % base_name)

    library_hdrs = hdrs + normal_hdrs + library_hdrs
    library_srcs = library_srcs + srcs
    CC_LIBRARY(
        name = name,
        srcs = library_srcs,
        hdrs = library_hdrs,
        deps = deps,
        copts = copts + ["-fPIC"],
        **kwargs
    )
