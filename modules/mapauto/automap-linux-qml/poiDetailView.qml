import QtQuick 2.0
import QtQuick.Controls 1.4
import QtQuick.Controls.Styles 1.4
Item {
    Connections {
        target:mapAuto;
        onPoiSelected: {
            poiSelected(name, address, distance);
        }
        onNoneSelected: {
            noneSelected();
        }
    }
    Connections {
        target:mapAuto;
        onNoneSelected:{
            poiRectangle.visible = false;
            locateBar.iconSource = "images/home_backparking.png";
            mapAuto.setMapMode(2);
        }
    }

    Rectangle {
        id:poiRectangle
        width:root.width
        height:root.height
        color:"#141414"
        visible: false
        Text {
            id:title
            text:"详情"
            font.family:"Heiti"
            font.pixelSize:20
            color:"white"
            anchors.topMargin: 10
            visible: true
            x:parent.x + 110
            y:55
        }
        Text {
            id:poiName
            font.family:"Heiti"
            font.pixelSize:20
            color:"white"
            y:105
        }
        Text {
            id:addr
            font.family:"Heiti"
            font.pixelSize:12
            color:"gray"
            y:poiName.y + 25
        }
        Button {
            id:faveritIcon
            property int selected : 0
            style: ButtonStyle {
                   background: Image {
                   }
            }
            x:poiName.x + 210
            y:poiName.y - 5
            width:35
            height:35
            iconSource:"images/poi_not_faverities.png"
            onClicked: {
                if(selected == 0) {
                    selected = 1;
                    faveritIcon.iconSource = "images/poi_faverities.png";
                } else {
                    selected = 0;
                    faveritIcon.iconSource = "images/poi_not_faverities.png";
                }
            }
        }

        Text {
            id:phone
            text:"暂无号码"
            font.family:"Heiti"
            font.pixelSize:20
            color:"white"
            y:poiName.y + 70
        }
        Button {
            id:phoneIcon
            style:ButtonStyle {
                background: Image {
                }
            }
            iconSource:"images/poi_phone.png"
            x:faveritIcon.x
            y:phone.y - 5
            onClicked: {
            }
        }
        Text {
            id:dist
            font.family:"Heiti"
            font.pixelSize:20
            color:"white"
            y:phone.y + 70
        }
        Button {
            id:navigateIcon
            style:ButtonStyle {
                background: Image {
                }
            }
            iconSource:"images/poi_navigate.png"
            x:faveritIcon.x
            y:dist.y - 5
            onClicked: {
            }
        }
    }

    function poiSelected(name, address, distance) {
        poiName.text = name;
        addr.text = address;
        if(distance > 1000) {
            var distanceValue = distance / 1000;
            distanceValue = distanceValue.toFixed(1);
            dist.text = "距离"+ distanceValue + "km";
        } else {
            dist.text = "距离"+ parseInt(distance) + "m";
        }
        poiRectangle.visible = true;
    }

    function noneSelected() {
        poiRectangle.visible = false;
    }
}
