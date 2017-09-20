import QtQuick 2.0
import QtQuick.Controls 1.4
import QtQuick.Controls.Styles 1.4
Item {
    Button {
        id:trafficBar
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
        iconSource :{source: getTrafficImageSource()}
        onClicked: {
             mapAuto.setTrafficSwitch(!mapAuto.getTrafficSwitch())
             trafficBar.iconSource = getTrafficImageSource()
         }


    }
    function getTrafficImageSource() {
        if(mapAuto.getTrafficSwitch()) {
            return ("source","images/home_roadcondition_on.png");
        } else {
            return ("source","images/home_roadcondition_off.png");
        }
    }

}
