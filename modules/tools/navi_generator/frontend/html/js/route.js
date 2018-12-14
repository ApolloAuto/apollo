
//Modification of Section Speed Limit Value
layui.define(['jquery','layer','coordinate','map'],function(exports){
        var $ = layui.jquery,
            layer = layui.layer,
            map = layui.map,
            coordinate = layui.coordinate;

    var obj = {
        planning:function (markerVal,bMap,type){
            var start = markerVal.get("start");
            var end = markerVal.get("end");
            if (start == undefined || end == undefined) {
                layer.msg("Please choose the starting point and the end point.", {icon: 5});
                return;
            } else {
                map.remove();
                obj.search(start, end, BMAP_DRIVING_POLICY_FIRST_HIGHWAYS,bMap,type);
            }
        },

        search:function(start, end, route,bMap,type){
            var driving = new BMap.DrivingRoute(bMap, {
                renderOptions: {map: bMap, autoViewport: true},
                policy: route,
                onSearchComplete: function (res) {
                    var s = coordinate.baiduApiToJson(start,end,driving,type);
                    if (!window.WebSocket) {
                        window.WebSocket = window.MozWebSocket;
                    };
                    if(socket.readyState!=1){
                        layer.msg("Error in connection establishment . ", {icon: 5});
                    }else{
                        socket.send(JSON.stringify(s));
                    }
                    
                }
            });
            driving.search(start, end);
        }
    };
    exports('route', obj);
});