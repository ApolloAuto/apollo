
add_library(Qt5::QSvgIconPlugin MODULE IMPORTED)

_populate_Svg_plugin_properties(QSvgIconPlugin RELEASE "iconengines/libqsvgicon.so")

list(APPEND Qt5Svg_PLUGINS Qt5::QSvgIconPlugin)
