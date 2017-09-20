import QtQuick 2.0

Item {
    Timer {
            interval: 1000; running: true; repeat: true
            onTriggered: time.text = getCurTime().toString()
    }
    function getCurTime()
    {
         var d = new Date();
         var hours = addZero(d.getHours());
         var minutes = addZero(d.getMinutes());
//             var seconds=addZero(d.getSeconds());
         var curTime =hours+":"+minutes;//+":"+seconds;
         return curTime;
    }

    function addZero(temp)
    {
         if(temp<10) return "0"+temp;
         else return temp;
    }
    Text {
        id: time
    }
 }
