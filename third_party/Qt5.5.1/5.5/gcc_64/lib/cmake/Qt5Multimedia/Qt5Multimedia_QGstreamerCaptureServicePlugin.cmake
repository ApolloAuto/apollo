
add_library(Qt5::QGstreamerCaptureServicePlugin MODULE IMPORTED)

_populate_Multimedia_plugin_properties(QGstreamerCaptureServicePlugin RELEASE "mediaservice/libgstmediacapture.so")

list(APPEND Qt5Multimedia_PLUGINS Qt5::QGstreamerCaptureServicePlugin)
