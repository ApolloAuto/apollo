import QtQuick 2.0
import QtQuick.Controls 1.4
import QtQuick.Controls.Styles 1.4
Item {
    Button {
        id:locateBar
        style: ButtonStyle {
               background: Rectangle {
                   implicitWidth: 35
                   implicitHeight: 35
                   border.width: control.pressed ?2 : 1
                   border.color: "white"
                   color:"white"
                   radius: 4
                   gradient: Gradient {
                       GradientStop { position: 0 ; color: control.pressed ? "#ccc" : "#eee" }
                       GradientStop { position: 1 ; color: control.pressed ? "#aaa" : "#ccc" }
                   }
               }
        }
        iconSource :{source:"images/home_north.png"}
        onClicked: {
            locate()
         }
    }
    function locate() {
       var AM_MAP_MODE_NORTH = 0
       var AM_MAP_MODE_DIRECT = 1
       var AM_MAP_MODE_NEED_FIX = 2
       if (mapAuto.getMapMode() == AM_MAP_MODE_NEED_FIX)
       {
           locateBar.iconSource = "images/home_north.png";
           mapAuto.setMapMode(AM_MAP_MODE_NORTH);
           mapAuto.setDefaultMapLocation();
       }
       else if (mapAuto.getMapMode() == AM_MAP_MODE_NORTH)
       {
           locateBar.iconSource = "images/home_orientation.png";
           mapAuto.setMapMode(AM_MAP_MODE_DIRECT);
       }
       else if (mapAuto.getMapMode() == AM_MAP_MODE_DIRECT)
       {
           locateBar.iconSource ="images/home_north.png";
           mapAuto.setMapMode(AM_MAP_MODE_NORTH);
       }
    }
}
