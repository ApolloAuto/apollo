
if (CMAKE_VERSION VERSION_LESS 2.8.3)
    message(FATAL_ERROR "Qt 5 requires at least CMake version 2.8.3")
endif()

get_filename_component(_qt5PrintSupport_install_prefix "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

# For backwards compatibility only. Use Qt5PrintSupport_VERSION instead.
set(Qt5PrintSupport_VERSION_STRING 5.5.1)

set(Qt5PrintSupport_LIBRARIES Qt5::PrintSupport)

macro(_qt5_PrintSupport_check_file_exists file)
    if(NOT EXISTS "${file}" )
        message(FATAL_ERROR "The imported target \"Qt5::PrintSupport\" references the file
   \"${file}\"
but this file does not exist.  Possible reasons include:
* The file was deleted, renamed, or moved to another location.
* An install or uninstall procedure did not complete successfully.
* The installation package was faulty and contained
   \"${CMAKE_CURRENT_LIST_FILE}\"
but not all the files it references.
")
    endif()
endmacro()

macro(_populate_PrintSupport_target_properties Configuration LIB_LOCATION IMPLIB_LOCATION)
    set_property(TARGET Qt5::PrintSupport APPEND PROPERTY IMPORTED_CONFIGURATIONS ${Configuration})

    set(imported_location "${_qt5PrintSupport_install_prefix}/lib/${LIB_LOCATION}")
    _qt5_PrintSupport_check_file_exists(${imported_location})
    set_target_properties(Qt5::PrintSupport PROPERTIES
        "INTERFACE_LINK_LIBRARIES" "${_Qt5PrintSupport_LIB_DEPENDENCIES}"
        "IMPORTED_LOCATION_${Configuration}" ${imported_location}
        "IMPORTED_SONAME_${Configuration}" "libQt5PrintSupport.so.5"
        # For backward compatibility with CMake < 2.8.12
        "IMPORTED_LINK_INTERFACE_LIBRARIES_${Configuration}" "${_Qt5PrintSupport_LIB_DEPENDENCIES}"
    )

endmacro()

if (NOT TARGET Qt5::PrintSupport)

    set(_Qt5PrintSupport_OWN_INCLUDE_DIRS "${_qt5PrintSupport_install_prefix}/include/" "${_qt5PrintSupport_install_prefix}/include/QtPrintSupport")
    set(Qt5PrintSupport_PRIVATE_INCLUDE_DIRS
        "${_qt5PrintSupport_install_prefix}/include/QtPrintSupport/5.5.1"
        "${_qt5PrintSupport_install_prefix}/include/QtPrintSupport/5.5.1/QtPrintSupport"
    )

    foreach(_dir ${_Qt5PrintSupport_OWN_INCLUDE_DIRS})
        _qt5_PrintSupport_check_file_exists(${_dir})
    endforeach()

    # Only check existence of private includes if the Private component is
    # specified.
    list(FIND Qt5PrintSupport_FIND_COMPONENTS Private _check_private)
    if (NOT _check_private STREQUAL -1)
        foreach(_dir ${Qt5PrintSupport_PRIVATE_INCLUDE_DIRS})
            _qt5_PrintSupport_check_file_exists(${_dir})
        endforeach()
    endif()

    set(Qt5PrintSupport_INCLUDE_DIRS ${_Qt5PrintSupport_OWN_INCLUDE_DIRS})

    set(Qt5PrintSupport_DEFINITIONS -DQT_PRINTSUPPORT_LIB)
    set(Qt5PrintSupport_COMPILE_DEFINITIONS QT_PRINTSUPPORT_LIB)
    set(_Qt5PrintSupport_MODULE_DEPENDENCIES "Widgets;Gui;Core")


    set(_Qt5PrintSupport_FIND_DEPENDENCIES_REQUIRED)
    if (Qt5PrintSupport_FIND_REQUIRED)
        set(_Qt5PrintSupport_FIND_DEPENDENCIES_REQUIRED REQUIRED)
    endif()
    set(_Qt5PrintSupport_FIND_DEPENDENCIES_QUIET)
    if (Qt5PrintSupport_FIND_QUIETLY)
        set(_Qt5PrintSupport_DEPENDENCIES_FIND_QUIET QUIET)
    endif()
    set(_Qt5PrintSupport_FIND_VERSION_EXACT)
    if (Qt5PrintSupport_FIND_VERSION_EXACT)
        set(_Qt5PrintSupport_FIND_VERSION_EXACT EXACT)
    endif()

    set(Qt5PrintSupport_EXECUTABLE_COMPILE_FLAGS "")

    foreach(_module_dep ${_Qt5PrintSupport_MODULE_DEPENDENCIES})
        if (NOT Qt5${_module_dep}_FOUND)
            find_package(Qt5${_module_dep}
                5.5.1 ${_Qt5PrintSupport_FIND_VERSION_EXACT}
                ${_Qt5PrintSupport_DEPENDENCIES_FIND_QUIET}
                ${_Qt5PrintSupport_FIND_DEPENDENCIES_REQUIRED}
                PATHS "${CMAKE_CURRENT_LIST_DIR}/.." NO_DEFAULT_PATH
            )
        endif()

        if (NOT Qt5${_module_dep}_FOUND)
            set(Qt5PrintSupport_FOUND False)
            return()
        endif()

        list(APPEND Qt5PrintSupport_INCLUDE_DIRS "${Qt5${_module_dep}_INCLUDE_DIRS}")
        list(APPEND Qt5PrintSupport_PRIVATE_INCLUDE_DIRS "${Qt5${_module_dep}_PRIVATE_INCLUDE_DIRS}")
        list(APPEND Qt5PrintSupport_DEFINITIONS ${Qt5${_module_dep}_DEFINITIONS})
        list(APPEND Qt5PrintSupport_COMPILE_DEFINITIONS ${Qt5${_module_dep}_COMPILE_DEFINITIONS})
        list(APPEND Qt5PrintSupport_EXECUTABLE_COMPILE_FLAGS ${Qt5${_module_dep}_EXECUTABLE_COMPILE_FLAGS})
    endforeach()
    list(REMOVE_DUPLICATES Qt5PrintSupport_INCLUDE_DIRS)
    list(REMOVE_DUPLICATES Qt5PrintSupport_PRIVATE_INCLUDE_DIRS)
    list(REMOVE_DUPLICATES Qt5PrintSupport_DEFINITIONS)
    list(REMOVE_DUPLICATES Qt5PrintSupport_COMPILE_DEFINITIONS)
    list(REMOVE_DUPLICATES Qt5PrintSupport_EXECUTABLE_COMPILE_FLAGS)

    set(_Qt5PrintSupport_LIB_DEPENDENCIES "Qt5::Widgets;Qt5::Gui;Qt5::Core")


    add_library(Qt5::PrintSupport SHARED IMPORTED)

    set_property(TARGET Qt5::PrintSupport PROPERTY
      INTERFACE_INCLUDE_DIRECTORIES ${_Qt5PrintSupport_OWN_INCLUDE_DIRS})
    set_property(TARGET Qt5::PrintSupport PROPERTY
      INTERFACE_COMPILE_DEFINITIONS QT_PRINTSUPPORT_LIB)

    _populate_PrintSupport_target_properties(RELEASE "libQt5PrintSupport.so.5.5.1" "" )




    file(GLOB pluginTargets "${CMAKE_CURRENT_LIST_DIR}/Qt5PrintSupport_*Plugin.cmake")

    macro(_populate_PrintSupport_plugin_properties Plugin Configuration PLUGIN_LOCATION)
        set_property(TARGET Qt5::${Plugin} APPEND PROPERTY IMPORTED_CONFIGURATIONS ${Configuration})

        set(imported_location "${_qt5PrintSupport_install_prefix}/plugins/${PLUGIN_LOCATION}")
        _qt5_PrintSupport_check_file_exists(${imported_location})
        set_target_properties(Qt5::${Plugin} PROPERTIES
            "IMPORTED_LOCATION_${Configuration}" ${imported_location}
        )
    endmacro()

    if (pluginTargets)
        foreach(pluginTarget ${pluginTargets})
            include(${pluginTarget})
        endforeach()
    endif()




_qt5_PrintSupport_check_file_exists("${CMAKE_CURRENT_LIST_DIR}/Qt5PrintSupportConfigVersion.cmake")

endif()
