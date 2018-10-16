
add_library(Qt5::QtQuick2Plugin MODULE IMPORTED)

_populate_Qml_plugin_properties(QtQuick2Plugin RELEASE "qmltooling/libqmldbg_qtquick2.so")

list(APPEND Qt5Qml_PLUGINS Qt5::QtQuick2Plugin)
