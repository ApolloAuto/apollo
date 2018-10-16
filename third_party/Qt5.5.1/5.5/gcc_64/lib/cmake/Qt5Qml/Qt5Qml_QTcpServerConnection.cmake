
add_library(Qt5::QTcpServerConnection MODULE IMPORTED)

_populate_Qml_plugin_properties(QTcpServerConnection RELEASE "qmltooling/libqmldbg_tcp.so")

list(APPEND Qt5Qml_PLUGINS Qt5::QTcpServerConnection)
