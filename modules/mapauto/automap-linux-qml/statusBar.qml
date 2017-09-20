import QtQuick 2.0

Item {
    // Clock Label
    Loader {
        id:timeBar
        source:"curTime.qml"
        x:parent.x
        y:parent.y
    }
    // wifi status icon
    Image {
        id:wifiBar
        fillMode: Image.PreserveAspectCrop
        source: "images/wifi_power.png"
        x:timeBar.x - 35
        y:timeBar.y - 14
      }
    // GPS status icon
    Image {
        id:gpsBar
        fillMode: Image.PreserveAspectCrop
        source: "images/gps_no.png"
        x:wifiBar.x - 35
        y:wifiBar.y
      }
  /*  Image {
        id:pBGMap;
        fillMode: Image.PreserveAspectCrop
        source:"images/1.jpg"
        x:0; y:0;
        //width:480; height:400;
    }*/
}
