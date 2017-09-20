import QtQuick 2.0
import QtQuick.Controls 1.4
import QtQuick.Controls.Styles 1.4
import QtQml 2.2

Item {
    /*setting Button*/
    Button {
        id:settingButton
        x:root.width / 4 + 20; y:40
        style: ButtonStyle {
            background: Rectangle {
                implicitWidth: 40
                implicitHeight: 40
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
        iconSource:"images/setting_setting.png"
        onClicked: {
            // TODO
        }
    }
    /*DatchDog Button*/
    Button {
        id:watchDogButton
        x:root.width / 4 + 20; y:100
        style: ButtonStyle {
            background: Rectangle {
                implicitWidth: 40
                implicitHeight: 40
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
        iconSource:"images/guide_ic_watch_dog_on.png"
        onClicked: {
          // TODO
        }
    }
    /*close Button*/
    Button {
        id:backbutton
        x:root.width / 4 + 20; y:root.height - 60
        style: ButtonStyle {
            background: Rectangle {
                implicitWidth: 40
                implicitHeight: 40
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
        iconSource:"images/guide_ic_close.png"
        onClicked: {
            guidanceClose();
        }
    }
    /*refresh Button*/
    Button {
        id:refreshButton
        x:root.width - 50; y:40
        width:40;height:40
        style: ButtonStyle {
            background: Rectangle {
                implicitWidth: 40
                implicitHeight: 40
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
        iconSource:"images/guide_ic_refresh.png"
        onClicked: {
          mapAuto.refreshRoute();
        }
    }

    Rectangle {
        id:guideLeftView
        color: "black"
        width:root.width / 4; height:root.height
        // Clock Label
        Timer {
                interval: 1000; running: true; repeat: true
                onTriggered: time.text = getCurTime().toString()
                function getCurTime()
                {
                     var d = new Date();
                     var hours = addZero(d.getHours());
                     var minutes = addZero(d.getMinutes());
                     var curTime =hours+":"+minutes;
                     return curTime;
                }
                function addZero(temp)
                {
                     if(temp<10) return "0"+temp;
                     else return temp;
                }
        }
        Text {
            id: time
            x: guideLeftView.width / 2 - time.width / 2;
            y:40;
            width:40;
            font.family:"Heiti"
            font.pixelSize:18
            color:"white"
        }
        /*Remin distance*/
        Text {
            id:remainDistLabel
            x:20; y:80
            font.family:"Heiti"
            font.pixelSize:18
            color:"white"
        }
        /*Reached Time*/
        Text {
            id:reachedTimeLabel
            x:guideLeftView.width / 2; y:80
            font.family:"Heiti"
            font.pixelSize:18
            color:"white"
        }
        /*Guidance image*/
        Image {
            id:guideImage
            fillMode: Image.PreserveAspectCrop
            width:150; height:150;
            x:guideLeftView.width / 2 - guideImage.width / 2; y:root.height / 2 - guideImage.height / 2 - 40;
        }
        /*Next turn Label*/
        Text {
            id:nextTurnLabel;
            x:guideLeftView.width / 2 - nextTurnLabel.width / 2; y:guideImage.y + guideImage.height;
            height:40;
            font.family:"Heiti"
            font.pixelSize:24
            color:"white"
        }
        /*Road name label*/
        Text {
            id:roadNameLabel;
            x:guideLeftView.width / 2 - roadNameLabel.width / 2; y:nextTurnLabel.y + nextTurnLabel.height;
            height:30;
            font.family:"Heiti"
            font.pixelSize:20
            color:"white"
        }
        /*Gps icon*/
        Image {
            id:gpsIamge;
            source:"images/gps_no.png"
            x:guideLeftView.width / 2 - gpsIamge.width / 2; y:root.height - 20 - gpsIamge.height / 2;
            width:30; height:30;
        }
        /*raster map*/
//        Image {
//            id:pBGMap;
//            source:"/home/l3/workspace/ENGINE/Gen5-HNavigation/1"
//            x:0; y:0;
//            width:480; height:400;
//        }

//        Rectangle {
//            id:canvasRect;
//            width:480;
//            height:400;

//                Canvas {
//                    id:myCanvas;
//                    width : 480;
//                    height : 400;
//                    contextType: "2d";
//                    renderStrategy: Canvas.Image;
//                    property var bdData: null;
//                    property var imageData:null;

//                    onPaint: {
//                        if(bdData != null) {
//                            //set image data
//                            var ctx = myCanvas.getContext("2d");
//                            imageData = ctx.createImageData(480, 400);
//                            var index = 0;
//                            for(var i = 0; i < 480*400*4; i+=4) {
//                                imageData.data[i] = bdData[index];
//                                imageData.data[i+1] = bdData[index+1];
//                                imageData.data[i+2] = bdData[index+2];
//                                var v1 = bdData[index];
//                                var v2 = bdData[index+1];
//                                var v3 = bdData[index+2];
//                                imageData.data[i+3] = 0;
//                                index += 3;
//                            }
//                            ctx.drawImage(imageData, 0 ,0);
//                        }
//                    }
//                }
//        }

        Connections {
            target:mapAuto
                onSetGuidanceData:{
                    //(QString guide_image_url, QString remain_distance, QString remain_time,  QString next_road_distance, QString next_road_name)
                    remainDistLabel.text = parseInt(remain_distance / 1000) + "公里";
                    reachedTimeLabel.text = remain_time
                    guideImage.source = guide_image_url;
                    roadNameLabel.text = next_road_name;
                    nextTurnLabel.text = parseInt(next_road_distance) +" 米后";
                }

                onDestinationArrived:{
                    guidanceClose();
                }

                onSetRasterMapInfoData:{
//                    myCanvas.bdData = pBDByteBuf;
//                    myCanvas.imageData = null;
//                    myCanvas.markDirty(Qt.rect(0, 0, 480, 400));
                }

        }        
    }
    function guidanceClose()
    {
        mapAuto.clearRoutes();
        mapAuto.stopNavigation();
        searchBar.visible = true;
        locateBar.visible = true;
        statusBar.visible = true;
        trafficImageBar.visible = true;
        if(stack.depth > 0)
        stack.clear();
    }
}
