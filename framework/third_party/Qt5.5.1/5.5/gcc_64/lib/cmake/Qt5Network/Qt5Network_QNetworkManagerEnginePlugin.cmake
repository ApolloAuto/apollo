
add_library(Qt5::QNetworkManagerEnginePlugin MODULE IMPORTED)

_populate_Network_plugin_properties(QNetworkManagerEnginePlugin RELEASE "bearer/libqnmbearer.so")

list(APPEND Qt5Network_PLUGINS Qt5::QNetworkManagerEnginePlugin)
