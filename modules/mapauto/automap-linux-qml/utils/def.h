#ifndef DEF_H
#define DEF_H

#include <QDebug>

#define NAMESPACE_START namespace AUTO_MAP {

#define NAMESPACE_END }

#define DELETE_SAFE(obj) if (obj) { delete obj; }

#define AM_LONG_PRESS_EXCEED_TIME (500) //ms
#define AM_DOUBLE_CLICK_EXCEED_TIME (200) //ms

#define AM_FRAME_PER_SECOND (24)
#define AM_MSEC_PER_SECOND (1000)

const int AM_LOCATION_SCALE_FACTOR = 100000;

typedef struct {
    QString uid;
    QString name;
    QString address;
    double longitude;
    double latitude;
} AMPoiInfo;

static const QString Setting_Key_Win_Width = "WIN_WIDTH";
static const QString Setting_Key_Win_Height = "WIN_HEIGHT";
static const QString Setting_Key_Traffic_Onoff = "TRAFFIC_ONOFF";
// 导航开始前的偏好设置项
static const QString Setting_Key_Prefer_Avoid_Jam = "PREFER_AVOID_JAM";
static const QString Setting_Key_Prefer_Highway_First = "PREFER_HIGHWAY_FIRST";
static const QString Setting_Key_Prefer_No_Highway = "PREFER_NO_HIGHWAY";
static const QString Setting_Key_Prefer_Less_Cost = "PREFER_LESS_COST";
// 导航过程中的偏好设置项
static const QString Setting_Key_Prefer_North_Heading = "PREFER_NORTH_HEADING";
static const QString Setting_Key_Watch_Dog_Onoff = "WATCH_DOG_ONOFF";
// 个人中心导航设置项
static const QString Setting_Key_UC_Prefer = "PREFER_UC";
static const QString Setting_Key_Prefer_Day_Night_Mode = "PREFER_UC_DAY_NIGHT_MODE";
static const QString Setting_Key_Prefer_Speak_Mode = "PREFER_UC_SPEAK_MODE";
static const QString Setting_Key_Prefer_Speak_Content = "PREFER_UC_SPEAK_CONTENT";
static const QString Setting_Key_Prefer_Auto_Cruise = "PREFER_UC_AUTO_CRUISE";
static const QString Setting_Key_Prefer_Route_Plan_Online_Prior = "PREFER_UC_ROUTE_PLAN_ONLINE_PRIOR";
static const QString Setting_Key_Prefer_Search_Online_Prior = "PREFER_UC_SEARCH_ONLINE_PRIOR";
// 主框检索历史
static const QString Setting_Key_History_List = "HISTORY_LIST";
static const QString Setting_Key_History_Item = "HISTORY_ITEM";

// UI边距配置
static const int Search_View_Head_Margin_Left = 10;
static const int Search_View_Head_Margin_Top = 30;
static const int Search_View_Head_Margin_Right = 0;
static const int Search_View_Head_Margin_Bottom = 0;

static const int Search_View_Body_Margin_Left = 60;
static const int Search_View_Body_Margin_Top = 10;
static const int Search_View_Body_Margin_Right = 120;
static const int Search_View_Body_Margin_Bottom = 20;
#endif // DEF_H
