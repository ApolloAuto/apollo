
add_library(Qt5::QAlsaPlugin MODULE IMPORTED)

_populate_Multimedia_plugin_properties(QAlsaPlugin RELEASE "audio/libqtaudio_alsa.so")

list(APPEND Qt5Multimedia_PLUGINS Qt5::QAlsaPlugin)
