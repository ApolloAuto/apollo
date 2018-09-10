
add_library(Qt5::QPSQLDriverPlugin MODULE IMPORTED)

_populate_Sql_plugin_properties(QPSQLDriverPlugin RELEASE "sqldrivers/libqsqlpsql.so")

list(APPEND Qt5Sql_PLUGINS Qt5::QPSQLDriverPlugin)
