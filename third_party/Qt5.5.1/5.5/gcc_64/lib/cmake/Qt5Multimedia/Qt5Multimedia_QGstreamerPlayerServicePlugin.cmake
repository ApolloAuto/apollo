
add_library(Qt5::QGstreamerPlayerServicePlugin MODULE IMPORTED)

_populate_Multimedia_plugin_properties(QGstreamerPlayerServicePlugin RELEASE "mediaservice/libgstmediaplayer.so")

list(APPEND Qt5Multimedia_PLUGINS Qt5::QGstreamerPlayerServicePlugin)
