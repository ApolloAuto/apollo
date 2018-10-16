
add_library(Qt5::QLinuxFbIntegrationPlugin MODULE IMPORTED)

_populate_Gui_plugin_properties(QLinuxFbIntegrationPlugin RELEASE "platforms/libqlinuxfb.so")

list(APPEND Qt5Gui_PLUGINS Qt5::QLinuxFbIntegrationPlugin)
