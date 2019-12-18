def _file_name(filePathName):
    if '/' in filePathName:
        return filePathName.rsplit('/', -1)[1]
    else:
        return filePathName

def _base_name(fileName):
    return fileName.split('.')[0]

def qt_cc_library(name, src, hdr, uis = [], res = [], normal_hdrs = [], deps = None, **kwargs):
    srcs = src
    for hItem in hdr:
        base_name = _base_name(_file_name(hItem))
        native.genrule(
            name = "%s_moc" % base_name,
            srcs = [hItem],
            outs = [ "moc_%s.cpp" % base_name ],
            cmd =  "if [[ `grep 'Q_OBJECT' $(location %s)` ]] ; \
            then /usr/lib/qt5/bin/moc $(location %s) -o $@ -f'%s'; \
            else echo '' > $@ ; fi"  % (hItem, hItem, '%s/%s' % (PACKAGE_NAME, hItem))
        )
        srcs.append("moc_%s.cpp" % base_name)

    for uitem in uis:
      base_name = _base_name(_file_name(uitem))
      native.genrule(
          name = "%s_ui" % base_name,
          srcs = [uitem],
          outs = ["ui_%s.h" % base_name],
          cmd = "/usr/lib/qt5/bin/uic $(locations %s) -o $@" % uitem,
      )
      hdr.append("ui_%s.h" % base_name)

    for ritem in res:
      base_name = _base_name(_file_name(ritem))
      native.genrule(
          name = "%s_res" % base_name,
          srcs = [ritem] + deps,
          outs = ["res_%s.cpp" % base_name],
          cmd = "/usr/lib/qt5/bin/rcc --name res --output $(OUTS) $(location %s)" % ritem,
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
