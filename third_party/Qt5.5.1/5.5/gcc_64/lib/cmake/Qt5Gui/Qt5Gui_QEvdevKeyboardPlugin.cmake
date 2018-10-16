
add_library(Qt5::QEvdevKeyboardPlugin MODULE IMPORTED)

_populate_Gui_plugin_properties(QEvdevKeyboardPlugin RELEASE "generic/libqevdevkeyboardplugin.so")

list(APPEND Qt5Gui_PLUGINS Qt5::QEvdevKeyboardPlugin)
