
add_library(Qt5::QConnmanEnginePlugin MODULE IMPORTED)

_populate_Network_plugin_properties(QConnmanEnginePlugin RELEASE "bearer/libqconnmanbearer.so")

list(APPEND Qt5Network_PLUGINS Qt5::QConnmanEnginePlugin)
