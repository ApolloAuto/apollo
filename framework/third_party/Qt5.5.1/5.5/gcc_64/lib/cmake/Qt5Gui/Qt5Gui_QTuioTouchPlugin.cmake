
add_library(Qt5::QTuioTouchPlugin MODULE IMPORTED)

_populate_Gui_plugin_properties(QTuioTouchPlugin RELEASE "generic/libqtuiotouchplugin.so")

list(APPEND Qt5Gui_PLUGINS Qt5::QTuioTouchPlugin)
