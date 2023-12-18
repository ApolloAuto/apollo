load("//tools:common.bzl", "dirname", "join_paths")
load("//tools/platform:common.bzl", "get_python_bin")

def _cyber_plugin_description(ctx):
    plugin_name = "__".join([
        ctx.attr.plugin.label.package.replace("/", "__"),
        ctx.attr.plugin.label.name,
    ])
    dst = join_paths("cyber_plugin_index", plugin_name)
    src = join_paths(ctx.attr.description.label.package, ctx.attr.description.label.name)
    out = ctx.actions.declare_file(dst)
    ctx.actions.write(out, src)
    return [DefaultInfo(files = depset([out]))]

cyber_plugin_description = rule(
    implementation = _cyber_plugin_description,
    attrs = {
        "plugin": attr.label(allow_files = True),
        "description": attr.label(allow_files = True),
    },
)
