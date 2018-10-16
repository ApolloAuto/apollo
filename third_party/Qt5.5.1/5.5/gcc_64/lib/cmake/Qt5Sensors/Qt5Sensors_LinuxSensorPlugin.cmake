
add_library(Qt5::LinuxSensorPlugin MODULE IMPORTED)

_populate_Sensors_plugin_properties(LinuxSensorPlugin RELEASE "sensors/libqtsensors_linuxsys.so")

list(APPEND Qt5Sensors_PLUGINS Qt5::LinuxSensorPlugin)
