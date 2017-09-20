import QtQuick 2.0
import QtQuick.Controls 1.4
import QtQuick.Controls.Styles 1.4
import QtQuick.Layouts 1.1

Item {
    Rectangle {
        color: "black"
        width:root.width
        height:root.height
        Button {
            id:backbutton
            property int selected : 0
            style: ButtonStyle {
                background: Image {
                    source:"images/back.png"
                    fillMode: Image.PreserveAspectCrop
                }
            }
            x:10; y:40
            width:30; height:30
            onClicked: {
                if(stack.depth > 0)
                stack.pop();
            }
        }
        Connections {
            target:mapAuto
                onSetSearchResultItemData:{
                    if(index == 0 || name == "") {
                        searchResultModel.clear();
                        return;
                    } else if(index == 1) {
                        searchResultModel.clear();
                    }

                    if(distance > 1000) {
                        var distanceValue = distance / 1000;
                        distanceValue = distanceValue.toFixed(1);
                        distanceValue = distanceValue + "km";
                    } else {
                        distanceValue =  parseInt(distance) + "m";
                    }
                    var listIndex = index + "";
                    var lng = longitude + "";
                    var lat = latitude + "";
                    searchResultModel.append({"index_numbuer":listIndex,"poiname":name,"address":address,"distance":distanceValue,"longitude":lng,"latitude":lat});
                }
                /*route plan result view*/
                onDisplayRouteplan:{
                    stack.push(routePlanPage);
                    mapAuto.width = root.width * 2 / 3;
                    loadingRouteplan.visible = false;
                    searchBar.visible = false;
                    locateBar.visible = false;
                    statusBar.visible = false;
                }
        }
        /*Routeplan View*/
        Component {
            id:routePlanPage;
            Loader {
                source:"routePlanView.qml"
            }
        }
        ListModel {
            id:searchResultModel

            ListElement {
                index_numbuer:""
                poiname:""
                address:""
                distance:""
                longitude:"0"
                latitude:"0"
            }
        }
        ListView {
            id:searchReultListView
            x:backbutton.x + 35;   y:backbutton.y + 10
            width: parent.width
            height: parent.height
            delegate: searchResultDelegate
            spacing:3
            orientation: ListView.Vertical
            model:searchResultModel
            highlight: Rectangle {               // color: "#00CED1";}
                width: 200; height: 50
                color: "#00CED1";
                radius: 5 ;
                y: searchReultListView.currentItem.y;
                Behavior on y { SpringAnimation { spring: 1; damping: 0.1 } }

            }
//            highlightFollowsCurrentItem: true;
            focus:true
        }
        Component {
            id:searchResultDelegate

            RowLayout {
                id:listElemnetID
                Text {
                    id:longitudeID
                    text:longitude
                    visible: false
                }
                Text {
                    id:latitudeID
                    text:latitude
                    visible: false
                }
                ColumnLayout {
                    id:column1
                    Text {
                        id:poiName
                        font.family:"Heiti"
                        height:35
                        font.pixelSize:20
                        color:"white"
                        text:index_numbuer + ". " + poiname
                    }
                    Text {
                        id:addr
                        font.family:"Heiti"
                        font.pixelSize:12
                        color:"gray"
                        text:address
                    }
                    Rectangle {
                        anchors.leftMargin: 35
                        y:addr.y + 40
                        width:root.width - 140
                        height: 1
                        color:"#1F2025"
                    }
                    states: State {
                        name: "Current"
                        when: listElemnetID.ListView.isCurrentItem
                        PropertyChanges { target: listElemnetID; x: 1 }
                    }
                    transitions: Transition {
                        NumberAnimation { properties: "x"; duration: 10 }
                    }
                    MouseArea {
                        anchors.fill: parent;
//                        width:root.width - 180;
//                        height:poiName.height //+ addr.height;
                        onClicked: {
                            listElemnetID.ListView.view.currentIndex = index;
                            // TODO  POI
                            mapAuto.focusPoiList(index);
                            if(stack.depth > 0) {
                                stack.clear();
                            }
                        }
                    }
                }

                ColumnLayout {
                    id:column2
                    Button {
                        id:navigateIcon
                        x:root.width - 120; y:poiName.y - 20;
                        style:ButtonStyle {
                            background: Rectangle {
                                implicitWidth: 35
                                implicitHeight: 35
                                border.width: control.pressed ?2 : 0
                                border.color: "blue"
                                color:"blue"
                                radius: 4
                                gradient: Gradient {
                                    GradientStop { position: 0 ; color: control.pressed ? "#ccc" : "transparent" }
                                    GradientStop { position: 1 ; color: control.pressed ? "#aaa" : "transparent" }
                                }
                            }
                        }
                        iconSource:"images/poi_navigate.png"
                        width: 35; height: 35
                        focus:true;
                        onClicked: {
                            loadingRouteplan.visible = true;
                            mapAuto.startRoutePlan(longitude, latitude);
                        }
                    }
                    Text {
                        id:dist
                        x:navigateIcon.x;
                        font.family:"Heiti"
                        font.pixelSize:12
                        color:"white"
                        text:distance
                    }
                }

            }
        }        
    }

    Text {
        id:loadingRouteplan
        width:parent.width; height:40
        x:300; y:160
        font.family:"Heiti"
        font.pixelSize:24
        color:"white"
        text:"正在进行导航路径规划，请稍后..."
        visible: false
    }

}
