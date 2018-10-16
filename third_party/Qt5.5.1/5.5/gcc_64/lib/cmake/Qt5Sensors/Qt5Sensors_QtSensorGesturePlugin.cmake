
add_library(Qt5::QtSensorGesturePlugin MODULE IMPORTED)

_populate_Sensors_plugin_properties(QtSensorGesturePlugin RELEASE "sensorgestures/libqtsensorgestures_plugin.so")

list(APPEND Qt5Sensors_PLUGINS Qt5::QtSensorGesturePlugin)
