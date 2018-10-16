
add_library(Qt5::QQuickWidgetPlugin MODULE IMPORTED)

_populate_Designer_plugin_properties(QQuickWidgetPlugin RELEASE "designer/libqquickwidget.so")

list(APPEND Qt5Designer_PLUGINS Qt5::QQuickWidgetPlugin)
