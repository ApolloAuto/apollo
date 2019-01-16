layui.define(['jquery', 'layer', 'coordinate', 'map', 'form'], function (exports) {
    var $ = layui.jquery,
        layer = layui.layer,
        form = layui.form,
        map = layui.map;

    var obj = {
        parseData: function (data, fileUpload) {
            if (data.type == "requestProcessBagFiles" || data.type == "requestCorrectRoadDeviation") {
                obj.existAbort(data, fileUpload);
            } else if (data.type == "requestSaveBagFiles" || data.type == "requestSaveRoadCorrection"
                || data.type == "requestSaveSpeedLimitCorrection") {
                layer.alert("Saved successfully", { icon: 1 });
            } else if (data.type == "requestModifySpeedLimit" || data.type == "requestRoute"
                || data.type == "requestNavigation") {
                obj.normalBack(data);
            } else if (data.type == "requestExcuteModuleCommand") {
                obj.moduleState(data);
            } else if (data.type == "SimWorldUpdate") {
                obj.recordPacket(data);
            } else if (data.type == "HMIStatus") {
                obj.hardware(data);
            } else {
                console.log("type error data is " + JSON.stringify(data));
            }
        },

        existAbort: function (data, fileUpload) {
            var type = data.type;
            if (data.result.success != 0 && data.result.success != 10000) {
                isOnabort = true;
                layer.open({
                    type: 1,
                    title: "Tips ",
                    skin: 'layui-layer-rim',
                    area: ['300px', '168px'],
                    content: '<div align="center" style="margin-top: 20px;padding:0 20px;">' + data.result.msg + '</div>',
                    btn: ['continue', 'cancel'],
                    closeBtn: 0
                    , yes: function () {
                        isOnabort = false;
                        layer.closeAll();
                        if (files.length > fileIdx) {
                            fileUpload.readFileSync(files[fileIdx], type);
                        }
                    }
                    , btn2: function () {
                        layer.closeAll();
                        for (var i = fileIdx; i < totalFileNum; i++) {
                            $("#fileIndex" + i).html("Transmission has been interrupted");
                        }
                        fileUpload.showButton();
                        fileUpload.saveButton(type);
                    }
                });
            } else {
                obj.renderPage(data);
            }
            if (data.result.success == 10000 || data.result.success == 10001) {
                if (type == "requestCorrectRoadDeviation") {
                    $("#deviationSave").removeClass("layui-btn-disabled").removeAttr("disabled");
                } else {
                    $("#preserve").removeClass("layui-btn-disabled").removeAttr("disabled");
                }
            }
        },

        moduleState: function (data) {
            var name = data.result.name;
            if (data.result.success != 0) {
                $("input[name='" + name + "']").prop('checked', false);
                form.render();
                layer.msg(data.result.msg, { icon: 5 });
            }
        },
        recordPacket: function (data) {
            map.createMarker(data);
        },
        hardware: function (data) {
            var GPS = data.data.hardware.GPS.summary;
            var CAN = data.data.hardware.CAN.summary;
            if (GPS == "OK") {
                $(".hardwareGPS .status-icon").css('background', '#13e453');
            } else {
                $(".hardwareGPS .status-icon").css('background', '#b43131');
            }
            if (CAN == "OK") {
                $(".hardwareCAN .status-icon").css('background', '#13e453');
            } else {
                $(".hardwareCAN .status-icon").css('background', '#b43131');
            }
            $(".hardwareGPS .gpsState").html(GPS);
            $(".hardwareCAN .canState").html(CAN);
        },
        normalBack: function (data) {
            if (data.result.success != 0) {
                layer.alert(data.result.msg, { icon: 5 });
            } else {
                obj.renderPage(data);
            }
        },

        addSpeedLimit: function (speedMin, speedMax, labLat, labLng) {
            var msg = "Speed: max " + speedMax;
            if (speedMin >= 0) {
                msg = msg + ", min " + speedMin;
            }
            var labelData = {
                lat: labLat,
                lng: labLng,
                msg: msg
            };
            if (labLat != -1 && labLng != -1) {
                map.addLabel(labelData);
            }
        },

        renderPage: function (data) {
            var resData = data.resData;
            var startPoint = resData.start;
            var endPoint = resData.end;
            var routePlans = resData.routePlans;

            for (let i = 0; i < routePlans.length; i++) {
                var routes = routePlans[i].routes;
                for (let j = 0; j < routes.length; j++) {
                    var navis = routes[j].navis;
                    for (let k = 0; k < navis.length; k++) {
                        var navi = navis[k];
                        var naviIndex = navi.naviIndex;
                        if (k == 0 && naviIndex == 0) {
                            var labLat = -1;
                            var labLng = -1;
                            if (navi.path.length > 0) {
                                labLat = navi.path[0].lat;
                                labLng = navi.path[0].lng;
                            }
                            var speedMax = routes[j].speedMax;
                            var speedMin = routes[j].speedMin;
                            obj.addSpeedLimit(speedMin, speedMax, labLat, labLng);
                        }
                        var path = navi.path;
                        map.mapRendering(path);
                    }
                }
            }
        }
    };
    exports('resolver', obj);
});