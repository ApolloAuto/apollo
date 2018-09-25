def _qt_ui_library(ui, deps):
    base_name = ui.split('.')[0]
    target_name = "%s_ui" % base_name

    print("ui: %s ---- %s" % (ui, target_name))

    native.genrule(
        name = target_name,
        srcs = [ui],
        outs = ["ui_%s" % ui],
        cmd = "uic $(locations %s) -o $@" % ui,
    )

def _qt_resource(res, deps):
    base_name = res.split('.')[0]
    target_name = "%s_res" % base_name

    print("res: %s ---- %s" % (res, target_name))

    native.genrule(
        name = target_name,
        srcs = [res] + deps,
        outs = ["res_%s" % res],
        cmd = "rcc --name $(OUTS) --output $(OUTS) $(location %s)" % res,
    )


def qt_cc_library(name, src, hdr, uis = [], res = [], normal_hdrs = [], deps = None, **kwargs):
    """Compiles a QT library and generates the MOC for it.
    If a UI file is provided, then it is also compiled with UIC.
    Args:
      name: A name for the rule.
      src: The cpp file to compile.
      hdr: The single header file that the MOC compiles to src.
      normal_hdrs: Headers which are not sources for generated code.
      deps: cc_library dependencies for the library.
      ui: If provided, a UI file to compile with UIC.
      ui_deps: Dependencies for the UI file.
      kwargs: Any additional arguments are passed to the cc_library rule.
    """
    # header_path = "%s/%s" % (PACKAGE_NAME, hdr) if len(PACKAGE_NAME) > 0 else hdr
   
    srcs = src
    for hItem in hdr:
        native.genrule(
            name = "%s_moc" % hItem.split('.')[0],
            srcs = [hItem],
            outs = [ "moc_%s.cpp" % hItem.split('.')[0] ],
            cmd =  "if [[ `grep 'Q_OBJECT' $(location %s)` ]] ; then /apollo/framework/third_party/Qt5.5.1/5.5/gcc_64/bin/moc $(location %s) -o $@ -f'%s'; else echo '' > $@ ; fi"  % (hItem, hItem, '%s/%s' % (PACKAGE_NAME, hItem))
        )
        srcs.append("moc_%s.cpp" % hItem.split('.')[0])

    for uitem in uis:
      base_name = uitem.split('.')[0]
      target_name = "%s_ui" % base_name

      native.genrule(
          name = target_name,
          srcs = [uitem],
          outs = ["ui_%s.h" % base_name],
          cmd = "/apollo/framework/third_party/Qt5.5.1/5.5/gcc_64/bin/uic $(locations %s) -o $@" % uitem,
      )
      hdr.append("ui_%s.h" % base_name)

    for ritem in res:
      base_name = ritem.split('.')[0]
      target_name = "%s_res" % base_name

      native.genrule(
          name = target_name,
          srcs = [ritem] + deps,
          outs = ["res_%s.cpp" % base_name],
          cmd = "/apollo/framework/third_party/Qt5.5.1/5.5/gcc_64/bin/rcc -o $(OUTS) $(location %s)" % ritem,
      )
      srcs.append("res_%s.cpp" % base_name)

    hdrs = hdr + normal_hdrs

    native.cc_library(
        name = name,
        srcs = srcs,
        hdrs = hdrs,
        deps = deps,
        **kwargs
    )