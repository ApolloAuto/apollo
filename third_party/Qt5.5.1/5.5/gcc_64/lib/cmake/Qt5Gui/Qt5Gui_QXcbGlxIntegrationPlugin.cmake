
add_library(Qt5::QXcbGlxIntegrationPlugin MODULE IMPORTED)

_populate_Gui_plugin_properties(QXcbGlxIntegrationPlugin RELEASE "xcbglintegrations/libqxcb-glx-integration.so")

list(APPEND Qt5Gui_PLUGINS Qt5::QXcbGlxIntegrationPlugin)
