layui.define(['jquery','layer','coordinate','map'],function(exports){
    var $ = layui.jquery,
        layer = layui.layer,
        map = layui.map;

    var obj = {
        parseData: function(data,fileUpload){
            if(data.type=="requestProcessBagFiles" ||data.type=="requestCorrectRoadDeviation"){
                obj.existAbort(data,fileUpload);
            }else if(data.type=="requestSaveBagFiles"||data.type=="requestSaveRoadCorrection"
                ||data.type=="requestSaveSpeedLimitCorrection"){
                layer.alert("Saved successfully", {icon: 1});
            }else if(data.type=="requestModifySpeedLimit"||data.type=="requestRoute"
                ||data.type=="requestNavigation"){
                obj.normalBack(data);
            }else{
                console.log("type error data is " + JSON.stringify(data));
            }
        },

        existAbort:function(data,fileUpload){
            if(data.result.success!=0){
                var type =  data.type;
                isOnabort = true;
                layer.open({
                    type: 1,
                    title:"error",
                    skin: 'layui-layer-rim',
                    area: ['360px', '220px'],
                    content: data.result.msg,
                    btn: ['continue', 'cancel'],
                    cancel : function(){
                        for (var i=fileIdx;i<totalFileNum;i++){
                            $("#fileIndex"+i).html("Transmission has been interrupted");
                        }
                    }
                    ,yes: function(){
                        isOnabort = false;
                        fileUpload.readFileSync(files[fileIdx]);
                    }
                    ,btn2: function(){
                        layer.closeAll();
                        isOnabort = false;
                        fileUpload.readFileSync(files[fileIdx]);
                    }
                });
            }else{
                obj.renderPage(data);
            }
		},

        normalBack :function(data){
            if(data.result.success!=0){
                layer.alert(data.result.msg, {icon: 5});
            }else{
                obj.renderPage(data);
            }
        },

        addSpeedLimit:function(speed_min,speed_max,labLat,labLng){
            var msg ="Speed: max "+speed_max;
            if(speed_min>=0){
                msg = msg + ", min "+speed_min;
            }
            var labelData={
                Lat: labLat,
                Lng: labLng,
                msg: msg
            };
            if(labLat!=-1 && labLng!=-1){
                map.addLabel(labelData);
            }
        },

        renderPage :function (data) {
            var resData = data.resData;
            var startPoint = resData.start;
            var endPoint  = resData.end;
            var routePlans = resData.routePlans;

            for (let i = 0; i < routePlans.length; i++){
                var routes = routePlans[i].routePlan.routes;
                for (let j = 0; j < routes.length; j++) {
                    var navis = routes[j].navis;
                    for (let k = 0; k < navis.length; k++) {
                        var navi = navis[k];
                        var naviIndex = navi.naviIndex;
                        if(k==0 && naviIndex==0){
                            var labLat =-1;
                            var labLng =-1;
                            if(navi.navi.path.length>0){
                                labLat = navi.navi.path[0].Lat;
                                labLng = navi.navi.path[0].Lng;
                            }
                            var speed_max = routes[j].speed_max;
                            var speed_min = routes[j].speed_min;
                            obj.addSpeedLimit(speed_min,speed_max,labLat,labLng);
                        }
                        var path = navi.navi.path;
                        map.mapRendering(path);
                    }
                }
            }
        }
    };
    exports('resolver', obj);
});