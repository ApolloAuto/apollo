
if (CMAKE_VERSION VERSION_LESS 3.0.0)
    message(FATAL_ERROR "Qt 5 UiPlugin module requires at least CMake version 3.0.0")
endif()

get_filename_component(_qt5UiPlugin_install_prefix "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)


macro(_qt5_UiPlugin_check_file_exists file)
    if(NOT EXISTS "${file}" )
        message(FATAL_ERROR "The imported target \"Qt5::UiPlugin\" references the file
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


if (NOT TARGET Qt5::UiPlugin)

    set(_Qt5UiPlugin_OWN_INCLUDE_DIRS "${_qt5UiPlugin_install_prefix}/include/" "${_qt5UiPlugin_install_prefix}/include/QtUiPlugin")
    set(Qt5UiPlugin_PRIVATE_INCLUDE_DIRS "")

    foreach(_dir ${_Qt5UiPlugin_OWN_INCLUDE_DIRS})
        _qt5_UiPlugin_check_file_exists(${_dir})
    endforeach()

    # Only check existence of private includes if the Private component is
    # specified.
    list(FIND Qt5UiPlugin_FIND_COMPONENTS Private _check_private)
    if (NOT _check_private STREQUAL -1)
        foreach(_dir ${Qt5UiPlugin_PRIVATE_INCLUDE_DIRS})
            _qt5_UiPlugin_check_file_exists(${_dir})
        endforeach()
    endif()

    set(_Qt5UiPlugin_MODULE_DEPENDENCIES "Widgets;Gui;Core")


    set(_Qt5UiPlugin_FIND_DEPENDENCIES_REQUIRED)
    if (Qt5UiPlugin_FIND_REQUIRED)
        set(_Qt5UiPlugin_FIND_DEPENDENCIES_REQUIRED REQUIRED)
    endif()
    set(_Qt5UiPlugin_FIND_DEPENDENCIES_QUIET)
    if (Qt5UiPlugin_FIND_QUIETLY)
        set(_Qt5UiPlugin_DEPENDENCIES_FIND_QUIET QUIET)
    endif()
    set(_Qt5UiPlugin_FIND_VERSION_EXACT)
    if (Qt5UiPlugin_FIND_VERSION_EXACT)
        set(_Qt5UiPlugin_FIND_VERSION_EXACT EXACT)
    endif()


    foreach(_module_dep ${_Qt5UiPlugin_MODULE_DEPENDENCIES})
        if (NOT Qt5${_module_dep}_FOUND)
            find_package(Qt5${_module_dep}
                5.5.1 ${_Qt5UiPlugin_FIND_VERSION_EXACT}
                ${_Qt5UiPlugin_DEPENDENCIES_FIND_QUIET}
                ${_Qt5UiPlugin_FIND_DEPENDENCIES_REQUIRED}
                PATHS "${CMAKE_CURRENT_LIST_DIR}/.." NO_DEFAULT_PATH
            )
        endif()

        if (NOT Qt5${_module_dep}_FOUND)
            set(Qt5UiPlugin_FOUND False)
            return()
        endif()

    endforeach()

    set(_Qt5UiPlugin_LIB_DEPENDENCIES "Qt5::Widgets;Qt5::Gui;Qt5::Core")


    add_library(Qt5::UiPlugin INTERFACE IMPORTED)

    set_property(TARGET Qt5::UiPlugin PROPERTY
      INTERFACE_INCLUDE_DIRECTORIES ${_Qt5UiPlugin_OWN_INCLUDE_DIRS})
    set_property(TARGET Qt5::UiPlugin PROPERTY
      INTERFACE_COMPILE_DEFINITIONS QT_UIPLUGIN_LIB)

    set_target_properties(Qt5::UiPlugin PROPERTIES
        INTERFACE_LINK_LIBRARIES "${_Qt5UiPlugin_LIB_DEPENDENCIES}"
    )

    file(GLOB pluginTargets "${CMAKE_CURRENT_LIST_DIR}/Qt5UiPlugin_*Plugin.cmake")

    macro(_populate_UiPlugin_plugin_properties Plugin Configuration PLUGIN_LOCATION)
        set_property(TARGET Qt5::${Plugin} APPEND PROPERTY IMPORTED_CONFIGURATIONS ${Configuration})

        set(imported_location "${_qt5UiPlugin_install_prefix}/plugins/${PLUGIN_LOCATION}")
        _qt5_UiPlugin_check_file_exists(${imported_location})
        set_target_properties(Qt5::${Plugin} PROPERTIES
            "IMPORTED_LOCATION_${Configuration}" ${imported_location}
        )
    endmacro()

    if (pluginTargets)
        foreach(pluginTarget ${pluginTargets})
            include(${pluginTarget})
        endforeach()
    endif()




_qt5_UiPlugin_check_file_exists("${CMAKE_CURRENT_LIST_DIR}/Qt5UiPluginConfigVersion.cmake")

endif()
