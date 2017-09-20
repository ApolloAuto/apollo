import QtQuick 2.0
import QtQuick.Controls 1.4
import QtQuick.Controls.Styles 1.4
Item {
    Button {
        id: searchEditButton
        style: ButtonStyle {
               background: Rectangle {
                   implicitWidth: 35
                   implicitHeight: 35
                   border.width: control.pressed ?2 : 1
                   border.color: "white"
                   color:"#CDBA96"
                   radius: 4
                   gradient: Gradient {
                       GradientStop { position: 0 ; color: control.pressed ? "#ccc" : "#eee" }
                       GradientStop { position: 1 ; color: control.pressed ? "#aaa" : "#ccc" }
                   }
               }
        }
        focus:true
        x:parent.x + 10
        y:parent.y + 8
        width: 400
        height: 40
        onClicked:{
            stack.push(searchPage);
        }
        TextEdit {
            id:searchText
            width:3
            height:parent.height
            text: "输入目的地"
            font.pointSize: 12
            color:"#CDBA96"
            x:searchEditButton.x + 3
            y:searchEditButton.y + 2
            focus:true
        }
    }
}
