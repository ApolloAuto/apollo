import QtQuick 2.2
import QtQuick.Window 2.2
import BaiduMapAuto 1.0
import QtQuick.Controls 1.4
import QtQuick.Controls.Styles 1.4
Window
{
    objectName: "window"
    id: root
    width: 1280
    height: 720
    visible: true
//    flags:"FramelessWindowHint"
    StackView {
        id:stack;
        anchors.centerIn: parent;
        width:root.width;
        height:root.height;
        property var home:null;


        /*mapAuto layer*/
        BaiduMapAutoQmlInterface
        {
            id: mapAuto
            anchors.fill: parent
//                x: 0; y:0;
//                width: root.width; height:root.height;
            ParallelAnimation
            {
                running: true
                NumberAnimation
                {
                    target: mapAuto
                    property: "x"
                    from: 0
                    to: 1024
                    duration: 40
                }
                loops: Animation.Infinite
            }
        }

        /*Mouse control*/
        PinchArea {
            anchors.fill : parent

            onPinchFinished: {
                if(pinch.scale < 1) {
                    mapAuto.zoomOut(true, 1);
                } else {
                    mapAuto.zoomIn(true, 1);
                }
            }
            Timer {
                id:longPressTimer
                interval:800
                repeat: false
                running:false
                onTriggered:{
                    mapAuto.focusAntiGeo(mouseArea.lastX, mouseArea.lastY);
                    longPressTimer.stop();
                }
            }

            MouseArea {
                id:mouseArea
                property int lastX
                property int lastY
                anchors.fill : parent
                acceptedButtons: Qt.LeftButton
                onPressed: {
                    lastX = mouseX;
                    lastY = mouseY;
                    longPressTimer.start();
                  //  mapAuto.focusPoi(mouseX, mouseY);
                 //   mapAuto.focusRoute(mouseX, mouseY);
                }
                onPositionChanged: {
                    mapAuto.mouseMove(lastX, lastY, mouse.x, mouse.y)
                    lastX = mouse.x;
                    lastY = mouse.y;
                    locateBar.iconSource = "images/home_backparking.png";
                    longPressTimer.stop();
                }
                onWheel:{
                        if(wheel.angleDelta.y > 0) {
                            mapAuto.zoomInHeightPrecision(0.1);
                        } else {
                            mapAuto.zoomOutHeightPrecision(0.1);
                        }
                        longPressTimer.stop()
                }
                onReleased: longPressTimer.stop()
            }
        }

        Timer {
            id:renderTimer
            repeat:true
            running:false
            interval: 41
            onTriggered:{
                mapAuto.render();
            }
        }

        Connections {
            target:mapAuto
                onStartRenderTimer:{
                    renderTimer.start();
                }
        }

        /*Search Bar*/
        Loader {
            id:searchBar
            source:"searchBar.qml"
            x:30
            y:40
            focus: true
        }

        /*Status Bar*/
        Loader {
            id:statusBar
            source:"statusBar.qml"
            x:root.width - 40
            y:searchBar.y + 10
        }

        /*Locate Bar*/
        Button {
            id:locateBar
            x:10
            y:root.height - 55
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


        /*Traffic Switch Bar*/
        Loader {
            id:trafficImageBar
            source:"trafficCondition.qml"
            x:root.width - 50
            y:zoomInBar.y - 50
            width:50
            height:50
        }

        /*Test Bar*/
//        Loader {
//            id:testBar
//            source:"test.qml"
//            x:trafficImageBar.x
//            y:trafficImageBar.y - 45

//        }
        /*ZoomIn Bar*/
        Loader {
            id:zoomInBar
            source:"zoomIn.qml"
            x:trafficImageBar.x
            y:zoomOutBar.y - 45

        }

        /*ZoomOut Bar*/
        Loader {
            id:zoomOutBar
            source:"zoomOut.qml"
            x:trafficImageBar.x
            y:root.height - 50
        }

        /*POI detail*/
        Loader {
            id:poiDetail
            width:root.width / 3
            height:root.height
            source:"poiDetailView.qml"
            x:root.width * 2 / 3
            y:0
            focus:true
        }

        /*test btn*/
        Button {
            visible: false
            id:testBtn
            x:trafficImageBar.x
            y:zoomOutBar.y - 190
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
            onClicked: {
                mapAuto.onTestBtn1();
             }
        }

        /*test btn2*/
        Button {
            visible: false
            id:testBtn2
            x:trafficImageBar.x
            y:zoomOutBar.y - 150
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
            onClicked: {
                mapAuto.onTestBtn2();
             }
        }
    }
    /*Search View*/
    Component {
        id:searchPage;
        Loader {
            id:searchView
            source:"searchView.qml"
        }
    }

    /*close Button*/
//    Button {
//        id:backbutton
//        x:root.width / 8; y:root.height - 60
//        style: ButtonStyle {
//            background: Rectangle {
//                implicitWidth: 40
//                implicitHeight: 40
//                border.width: control.pressed ?2 : 1
//                border.color: "white"
//                color:"white"
//                radius: 4
//                gradient: Gradient {
//                    GradientStop { position: 0 ; color: control.pressed ? "#ccc" : "#eee" }
//                    GradientStop { position: 1 ; color: control.pressed ? "#aaa" : "#ccc" }
//                }
//            }
//        }
//        iconSource:"images/guide_ic_close.png"
//        onClicked: {
//           root.close();
//        }
//    }

//    //for test canvas
//    Rectangle {
//        id:canvasRect;
//        width:480;
//        height:400;

//            Canvas {
//                id:myCanvas;
//                width : 480;
//                height : 400;
//                contextType: "2d";
//                property var imageData:null;

//                onPaint: {
//                    if(imageData == null) {
//                        var ctx = myCanvas.getContext("2d");
//                        imageData = ctx.createImageData(120, 100);
//                        for(var i = 0; i < 12000; i+=4) {
//                            imageData.data[i] = Math.floor(Math.random() *255);
//                            imageData.data[i+1] = Math.floor(Math.random() *255);
//                            imageData.data[i+2] = Math.floor(Math.random() *255);
//                            imageData.data[i+3] = Math.floor(Math.random() *255);
//                        }
//                        ctx.drawImage(imageData, 80 ,100);
//                    }
//                }
//            }
//    }

}

