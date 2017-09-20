import QtQuick 2.0
import QtQuick.Controls 1.4
import QtQuick.Controls.Styles 1.4
Item {
    Button {
        id:zoomout
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
        iconSource :{source: "images/home_zoom_out.png"}
        onClicked: {
            mapAuto.zoomOut(true, 1);
         }
    }
}
