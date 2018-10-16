
add_library(Qt5::QGenericEnginePlugin MODULE IMPORTED)

_populate_Network_plugin_properties(QGenericEnginePlugin RELEASE "bearer/libqgenericbearer.so")

list(APPEND Qt5Network_PLUGINS Qt5::QGenericEnginePlugin)
