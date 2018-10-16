
add_library(Qt5::QMinimalIntegrationPlugin MODULE IMPORTED)

_populate_Gui_plugin_properties(QMinimalIntegrationPlugin RELEASE "platforms/libqminimal.so")

list(APPEND Qt5Gui_PLUGINS Qt5::QMinimalIntegrationPlugin)
