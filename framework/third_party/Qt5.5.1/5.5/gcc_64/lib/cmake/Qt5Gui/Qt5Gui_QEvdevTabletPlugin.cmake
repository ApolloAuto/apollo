
add_library(Qt5::QEvdevTabletPlugin MODULE IMPORTED)

_populate_Gui_plugin_properties(QEvdevTabletPlugin RELEASE "generic/libqevdevtabletplugin.so")

list(APPEND Qt5Gui_PLUGINS Qt5::QEvdevTabletPlugin)
