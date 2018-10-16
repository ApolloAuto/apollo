
add_library(Qt5::QGtk2ThemePlugin MODULE IMPORTED)

_populate_Gui_plugin_properties(QGtk2ThemePlugin RELEASE "platformthemes/libqgtk2.so")

list(APPEND Qt5Gui_PLUGINS Qt5::QGtk2ThemePlugin)
