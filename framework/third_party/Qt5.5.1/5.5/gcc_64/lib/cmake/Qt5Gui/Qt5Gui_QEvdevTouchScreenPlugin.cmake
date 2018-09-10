
add_library(Qt5::QEvdevTouchScreenPlugin MODULE IMPORTED)

_populate_Gui_plugin_properties(QEvdevTouchScreenPlugin RELEASE "generic/libqevdevtouchplugin.so")

list(APPEND Qt5Gui_PLUGINS Qt5::QEvdevTouchScreenPlugin)
