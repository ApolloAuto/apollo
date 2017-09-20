import QtQuick 2.0
import QtQuick.Controls 1.4
import QtQuick.Controls.Styles 1.4
import QtQuick.Layouts 1.1

Item {
    /*back button*/
    Button {
        id:backbutton
        x:10; y:40
        width:40; height:40
        property int selected : 0
        focus:true;
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
        iconSource:"images/back.png"
        onClicked: {
            mapAuto.cancelRoutePlan();
            mapAuto.clearRoutes();
            if(stack.depth > 0)
            stack.pop();
        }
    }
    Connections {
        target:mapAuto
            onSetRoutePlanResultItemData:{
                if(index == 1) {
                    routeplanModel.clear();
                }

                var timevalue;
                if(time > 3600) {
                    if(time % 3600 == 0) {
                        timevalue = parseInt(time / 3600) + "小时";
                    } else {
                        timevalue = parseInt(time / 3600) + "小时" + parseInt((time % 3600)/60) + "分钟";
                    }
                } else {
                    timevalue = parseInt(time / 60) + "分钟";
                }

                var indexvalue;
                switch (index) {
                case 1:
                    indexvalue = "用时较少";
                    break;
                case 2:
                    indexvalue = "方案二";
                    break;
                case 3:
                    indexvalue = "方案三";
                    break;
                default:
                    break;
                }
                var lengthvalue
                if (length < 1000) {
                    lengthvalue = length + "米";
                }
                else {
                    lengthvalue =parseInt(length / 1000) + "公里";
                }
                var trafficSignalCountValue = ("红绿灯" + traffic_signal_count + "个");
                routeplanModel.append({"time":timevalue,"method_index":indexvalue,"length":lengthvalue,"trafficSignalCount":trafficSignalCountValue});
            }
    }
    /*routeplan result list*/
    Rectangle {
        color: "black"
        x:root.width *2/3
        width:root.width/3
        height:root.height

        ListModel {
            id:routeplanModel
            ListElement {
                time:" "
                method_index:" "
                length:" "
                trafficSignalCount:""
            }
//            ListElement {
//                time:"20分钟"
//                method_index:"方案一"
//                length:"60km"
//                trafficSignalCount:"红绿灯70个"
//            }
//            ListElement {
//                time:"30分钟"
//                method_index:"方案二"
//                length:"30km"
//                trafficSignalCount:"红绿灯60个"
//            }
        }
        ListView {
            id:routeplanListView
            y:40
            width: parent.width
            height: parent.height
            delegate: routeplanDelegate
            spacing:50
            orientation: ListView.Vertical
            model:routeplanModel
            highlight: Rectangle {
                width: 200; height: 50
                color: "#00CED1";//"lightsteelblue";
                radius: 5 ;
                y: routeplanListView.currentItem.y;
                Behavior on y { SpringAnimation { spring: 2; damping: 0.1 } }

            }
            highlightFollowsCurrentItem: true;
            focus:true            
        }
        Component {
            id:routeplanDelegate
            RowLayout {
                id:idlistElemnet
                ColumnLayout {
                    id:column1
                    Text {
                        id:timeID
                        font.family:"Heiti"; font.pixelSize:22
                        color:"white"
                        text:time
                    }
                    RowLayout {
                        Text {
                            id:method_indexID
                            font.family:"Heiti"; font.pixelSize:14
                            color:"gray"
                            text:method_index
                        }
                        Text {
                            id:lengthID
                            x:method_indexID.x + 110; y:method_indexID.y
                            font.family:"Heiti"; font.pixelSize:14
                            color:"gray"
                            text:length
                        }
                        Text {
                            id:trafficSignalCountID
                            x:lengthID.x + 100; y:lengthID.y
                            font.family:"Heiti"; font.pixelSize:14
                            color:"gray"
                            text:trafficSignalCount
                        }
                    }
                    Rectangle {
                        anchors.leftMargin: 35
                        width:root.width - 80
                        height: 1
                        color:"#1F2025"
                    }
                }

                states: State {
                    name: "Current"
                    when: idlistElemnet.ListView.isCurrentItem
                    PropertyChanges { target: idlistElemnet; x: 1 }
                }
                transitions: Transition {
                    NumberAnimation { properties: "x"; duration: 10 }
                }
                MouseArea {
                    anchors.fill: parent
                    onClicked: {
                        idlistElemnet.ListView.view.currentIndex = index;
                        mapAuto.selectRouteByID(index);
                    }
                }
            }
        }
        Button {
            x:70
            y:root.height - 100
            width:140; height:40
            style: ButtonStyle {
                   background: Rectangle {
                       implicitWidth: 140
                       implicitHeight: 40
                       border.width: control.pressed ?2 : 1
                       border.color: "blue"
                       color:"blue"
                       radius: 4
                       gradient: Gradient {
                           GradientStop { position: 0 ; color: control.pressed ? "#ccc" : "lightblue" }
                           GradientStop { position: 1 ; color: control.pressed ? "#aaa" : "lightblue" }
                       }
                   }
            }
            text:"开始导航"
            onClicked: {
//                mapAutoView.x = root.width / 4;
//                mapAutoView.width = root.width * 3 / 4;
                mapAuto.startGuide();
                trafficImageBar.visible = false;
                stack.push(routeGuidePage);
            }
        }
    }
    /*Routeplan View*/
    Component {
        id:routeGuidePage;
        Loader {
            source:"routeGuideView.qml"
        }
    }

}
