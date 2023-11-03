load("//tools/install:install.bzl", "install", "install_files", "install_src_files", "install_plugin")
load("@rules_cc//cc:defs.bzl", "cc_library", "cc_binary", "cc_test")
load("//tools:apollo.bzl", "cyber_plugin_description")

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
            "type": "disable_source"
        } 
    header_action = {
        "name": "install_header_src",
        "src_dir": ["."],
        "dest": "include/%s" % package_name,
        "filter": "*.h",
        "type": "neo",
    }

    _add_install_rules(
        package_install_target, [src_code_action, header_action],
        has_install_rule, has_install_src_rule, package_name,
        subpackages_install_target, subpackages_install_src_target)

def apollo_cc_test(**kwargs):
    # simple wrap for cc_test
    cc_test(**kwargs)

def apollo_cc_binary(**kwargs):
    # simple wrap for cc_binary
    cc_binary(**kwargs)

def apollo_component(**kwargs):
    if not kwargs["name"].startswith("lib") or not kwargs["name"].endswith(".so"):
        fail("name must start with 'lib' and end with '.so'")
    if "alwayslink" in kwargs:
        fail("'apollo_component' macro has not 'alwayslink' attribute")

    internal_lib_name = "{}_lib".format(kwargs["name"][3: len(kwargs["name"])-3])

    apollo_cc_library(**dict(kwargs, name = internal_lib_name, 
                            visibility = ["//visibility:public"]))
    
    cc_binary(
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
        
    cc_binary(**dict(
        bin_kwargs, name = "lib{}.so".format(bin_kwargs["name"]),
        linkshared = True, linkstatic = True, srcs = merge_src,
        visibility = ["//visibility:public"], tags = ["export_library", kwargs["name"]]))
    cc_library(**dict(
        kwargs, srcs = [":lib{}.so".format(kwargs["name"])],
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

    cc_library(
        name = cc_library_name,
        srcs = kwargs["srcs"] if "srcs" in kwargs else [],
        hdrs = kwargs["hdrs"] if "hdrs" in kwargs else [],
        deps = kwargs["deps"] if "deps" in kwargs else [],
        copts = kwargs["copts"] if "copts" in kwargs else [], 
        alwayslink = True,
    )

    cc_binary(
        name = kwargs["name"],
        linkshared = True,
        linkstatic = True,
        deps = [":{}".format(cc_library_name)],
    )