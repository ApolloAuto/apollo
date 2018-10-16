
add_library(Qt5::QCupsPrinterSupportPlugin MODULE IMPORTED)

_populate_PrintSupport_plugin_properties(QCupsPrinterSupportPlugin RELEASE "printsupport/libcupsprintersupport.so")

list(APPEND Qt5PrintSupport_PLUGINS Qt5::QCupsPrinterSupportPlugin)
