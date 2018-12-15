
//Modification of Section Speed Limit Value
layui.define(['jquery','layer','map'],function(exports){
        var $ = layui.jquery,
            layer = layui.layer,
            map = layui.map;

    var obj = {
        preserve:function (start,end){
            var minSpeed=$("#modify-minSpeed").val();
            var maxSpeed=$("#modify-maxSpeed").val();

            var reg = /(^[0-9]$)|(^[1-9][0-9]$)|(^[1][0-9]{0,2}$)/;
            if(!reg.test(minSpeed) || !reg.test(maxSpeed)){
                layer.msg('minSpeed and maxSpeed has to be between 0 and 199', {icon: 5});
                return;
            }
            minSpeed = parseInt(minSpeed);
            maxSpeed = parseInt(maxSpeed);
            if(minSpeed>maxSpeed){
                layer.msg('maxSpeed can not be less than minSpeed', {icon: 5});
                return;
            }

            var startPoint =start.point;
            var endPoint = end.point;

            var resultMap = {
                "type":"requestModifySpeedLimit",
                "start":startPoint,
                "end":endPoint,
                "speed_min":minSpeed,
                "speed_max":maxSpeed
            };
            if (!window.WebSocket) window.WebSocket = window.MozWebSocket;

            if(socket.readyState!=1){
                layer.msg("Error in connection establishment .", {icon: 5});
            }else{
                socket.send(JSON.stringify(resultMap));
                layer.msg("The modify Speed instruction has been sent", {icon: 1});
                $("#speedSave").removeClass("layui-btn-disabled");
                map.remove();
            }   
        },

        pointSelect:function(markerVal){
            var start = markerVal.get("start");
            var end = markerVal.get("end");
            if (start == undefined || end == undefined) {
                layer.msg("Please choose the starting point and the end point.", {icon: 5});
                return;
            } else {
                obj.preserve(start,end);
            }
        },
    };
    exports('modifySpeed', obj);
});