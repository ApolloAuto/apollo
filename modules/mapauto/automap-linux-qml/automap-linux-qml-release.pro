TEMPLATE = app
TARGET = automap-linux-qml
INCLUDEPATH += . \
    ./include \
    ./include/BDCommon \
    ./include/BDSearch \
    ./include/BDMapView \
    ./include/BDRouteGenerator \
    ./include/BDRouteGuide \
    ./include/BDLocation \
    ./include/BDMapMatching

QT += qml quick core network quickwidgets

CONFIG += c++11
#CONFIG += console

LIBS += -lGLESv2
LIBS += -L../automap-linux-qml/libs/curl -lcurl -Wl,--allow-shlib-undefined
LIBS += -L../automap-linux-qml/libs -lanticheat
LIBS += -L../automap-linux-qml/libs -lvi
LIBS += -L../automap-linux-qml/libs -lengine
LIBS += -L../automap-linux-qml/libs -lBaiduMapEngine
LIBS += -L../automap-linux-qml/libs -lwordseglite
LIBS += -L../automap-linux-qml/libs -lMapEngineInterface

SOURCES += main.cpp \
    src/BaiduMapAutoQmlInterface.cpp \
    utils/amsettingmanager.cpp \
    utils/amgeotools.cpp \

RESOURCES += qml.qrc
# Additional import path used to resolve QML modules in Qt Creator's code model
QML_IMPORT_PATH =
DEFINES =  \
            \#__V_LINUX_ARM__ \ #define arm platform
            __V_LINUX_PC__
# Default rules for deployment.

HEADERS += \
    src/BaiduMapAutoQmlInterface.h \
    utils/amsettingmanager.h \
    utils/amgeotools.h \
    engineSdk/amrouteguideengine.h \
    include/BNAObserver/IAdapterMsgObserver.h \
    include/BNATools/navi_car_adapter_def.h \
    include/BNATools/navi_auto_location_def.h \
    include/BNATools/navi_auto_guidance_def.h \
    include/BNATools/navi_auto_threadpool.h \
    include/BNATools/navi_car_base.h \
    include/BNAInterface/navi_auto_adapter_if.h

DISTFILES += \
    Shader.vsh \
    shader.fsh \
    images/back.png \
    images/bnav_common_check_box_checked.png \
    images/bnav_common_check_box_checked_night.png \
    images/bnav_common_check_box_unchecked.png \
    images/bnav_common_check_box_unchecked_night.png \
    images/bnav_poi_detail_ic_down_normal.png \
    images/bnav_poi_detail_ic_faverities.png \
    images/bnav_poi_detail_ic_not_faverities.png \
    images/clear.png \
    images/com_loading.png \
    images/gps_1.png \
    images/gps_2.png \
    images/gps_3.png \
    images/gps_no.png \
    images/guide_ic_close.png \
    images/guide_ic_refresh.png \
    images/guide_ic_watch_dog_off.png \
    images/guide_ic_watch_dog_on.png \
    images/home_backparking.png \
    images/home_more.png \
    images/home_north.png \
    images/home_orientation.png \
    images/home_roadcondition_off.png \
    images/home_roadcondition_on.png \
    images/home_zoom_in.png \
    images/home_zoom_out.png \
    images/left.png \
    images/map_ic_navigating_set_selected.png \
    images/map_ic_navigating_set_unchecked.png \
    images/map_ic_offline_continue.png \
    images/map_ic_offline_delete.png \
    images/map_ic_offline_download.png \
    images/map_ic_offline_pause.png \
    images/map_ic_oil.png \
    images/map_ic_park.png \
    images/map_ic_toilets.png \
    images/map_ic_wash.png \
    images/nsdk_drawable_rg_hud_turn_back.png \
    images/nsdk_drawable_rg_hud_turn_branch_center.png \
    images/nsdk_drawable_rg_hud_turn_branch_left.png \
    images/nsdk_drawable_rg_hud_turn_branch_left_straight.png \
    images/nsdk_drawable_rg_hud_turn_branch_right.png \
    images/nsdk_drawable_rg_hud_turn_branch_right_straight.png \
    images/nsdk_drawable_rg_hud_turn_dest.png \
    images/nsdk_drawable_rg_hud_turn_front.png \
    images/nsdk_drawable_rg_hud_turn_left.png \
    images/nsdk_drawable_rg_hud_turn_left_2branch_left.png \
    images/nsdk_drawable_rg_hud_turn_left_2branch_right.png \
    images/nsdk_drawable_rg_hud_turn_left_3branch_left.png \
    images/nsdk_drawable_rg_hud_turn_left_3branch_middle.png \
    images/nsdk_drawable_rg_hud_turn_left_3branch_right.png \
    images/nsdk_drawable_rg_hud_turn_left_back.png \
    images/nsdk_drawable_rg_hud_turn_left_side.png \
    images/nsdk_drawable_rg_hud_turn_left_side_ic.png \
    images/nsdk_drawable_rg_hud_turn_left_side_main.png \
    images/nsdk_drawable_rg_hud_turn_right.png \
    images/nsdk_drawable_rg_hud_turn_right_2branch_left.png \
    images/nsdk_drawable_rg_hud_turn_right_2branch_right.png \
    images/nsdk_drawable_rg_hud_turn_right_3branch_left.png \
    images/nsdk_drawable_rg_hud_turn_right_3branch_middle.png \
    images/nsdk_drawable_rg_hud_turn_right_3branch_right.png \
    images/nsdk_drawable_rg_hud_turn_right_back.png \
    images/nsdk_drawable_rg_hud_turn_right_front.png \
    images/nsdk_drawable_rg_hud_turn_right_side.png \
    images/nsdk_drawable_rg_hud_turn_right_side_ic.png \
    images/nsdk_drawable_rg_hud_turn_right_side_main.png \
    images/nsdk_drawable_rg_hud_turn_tollgate.png \
    images/poi_faverities.png \
    images/poi_navigate.png \
    images/poi_not_faverities.png \
    images/poi_phone.png \
    images/poi_pick.png \
    images/right.png \
    images/route_prefer_close.png \
    images/route_prefer_close_press.png \
    images/search_input.png \
    images/setting_collection.png \
    images/setting_data.png \
    images/setting_exit.png \
    images/setting_help.png \
    images/setting_icon.png \
    images/setting_login.png \
    images/setting_nearby.png \
    images/setting_setting.png \
    images/wifi_no.png \
    images/wifi_power.png \
    images/wifi_weak.png \
    qml/MainForm.ui.qml \
    qml/curTime.qml \
    qml/main.qml
