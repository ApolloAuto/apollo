def qt_cc_library(name, src, hdr, uis = [], res = [], normal_hdrs = [], deps = None, **kwargs):  
    srcs = src
    for hItem in hdr:
        native.genrule(
            name = "%s_moc" % hItem.split('.')[0],
            srcs = [hItem],
            outs = [ "moc_%s.cpp" % hItem.split('.')[0] ],
            cmd =  "if [[ `grep 'Q_OBJECT' $(location %s)` ]] ; \
            then /apollo/framework/third_party/Qt5.5.1/5.5/gcc_64/bin/moc $(location %s) -o $@ -f'%s'; \
            else echo '' > $@ ; fi"  % (hItem, hItem, '%s/%s' % (PACKAGE_NAME, hItem))
        )
        srcs.append("moc_%s.cpp" % hItem.split('.')[0])

    for uitem in uis:
      base_name = uitem
      if '/' in uitem:
          base_name = uitem.rsplit('/', -1)[1];
      base_name = base_name.split('.')[0]
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
          cmd = "/apollo/framework/third_party/Qt5.5.1/5.5/gcc_64/bin/rcc --name res --output $(OUTS) $(location %s)" % ritem,
      )
      srcs.append("res_%s.cpp" % base_name)

    hdrs = hdr + normal_hdrs

    native.cc_library(
        name = name,
        srcs = srcs,
        hdrs = hdrs,
        deps = deps,
        alwayslink = 1,
        **kwargs
    )