
//Map leading line drawing
layui.define(['jquery', 'layer', 'upload', 'coordinate'], function (exports) {
    var $ = layui.jquery,
        layer = layui.layer,
        upload = layui.upload,
        coordinate = layui.coordinate;
    var map = new BMap.Map('allmap');
    var markerVal = new Map();
    var startmyIcon = new BMap.Icon("images/start.png", new BMap.Size(32, 43));
    var waypointmyIcon = new BMap.Icon("images/pathPoint.png", new BMap.Size(32, 43));
    var endmyIcon = new BMap.Icon("images/end.png", new BMap.Size(32, 43));
    var myIcon = new BMap.Icon("http://lbsyun.baidu.com/jsdemo/img/car.png", new BMap.Size(52, 26));
    var oldPoint;
    var obj = {
        map: function () {
            var point = new BMap.Point(112.885542, 28.122144);
            var geoc = new BMap.Geocoder();
            map.centerAndZoom(point, 16);
            map.addControl(new BMap.OverviewMapControl());       //Adding abbreviated map
            map.enableScrollWheelZoom();                         //Enable rollers to zoom in and out
            map.addControl(new BMap.NavigationControl());
            map.disable3DBuilding();
            changeMapStyle('dark')  //Custom map color
            function changeMapStyle(style) {
                map.setMapStyle({ style: style });
            }
            //Create right menu
            var menu = new BMap.ContextMenu();
            var txtMenuItem = [
                {
                    text: 'Starting point',
                    callback: function (e) {
                        if (markerVal.get("start")) {
                            map.removeOverlay(markerVal.get("start"));
                        }
                        var pt = new BMap.Point(parseFloat(e.lng), parseFloat(e.lat));
                        var marker = new BMap.Marker(pt, { icon: startmyIcon });  // Create annotation
                        marker.enableDragging();
                        markerVal.set("start", marker);
                        map.addOverlay(marker);
                        geoc.getLocation(pt, function (rs) {
                            var addComp = rs.addressComponents;
                            var site = addComp.province + addComp.city + addComp.district + addComp.street + addComp.streetNumber;
                            $(".route-start input").val(site);
                        });
                    }
                },
                {
                    text: 'End point',
                    callback: function (e) {
                        if (markerVal.get("end")) {
                            map.removeOverlay(markerVal.get("end"));
                        }
                        var pt = new BMap.Point(parseFloat(e.lng), parseFloat(e.lat));
                        var marker3 = new BMap.Marker(pt, { icon: endmyIcon });
                        markerVal.set("end", marker3);
                        marker3.enableDragging();
                        map.addOverlay(marker3);
                        geoc.getLocation(pt, function (rs) {
                            var addComp = rs.addressComponents;
                            var site = addComp.province + addComp.city + addComp.district + addComp.street + addComp.streetNumber;
                            $(".route-end input").val(site)
                        });
                    }
                }
            ];
            for (var i = 0; i < txtMenuItem.length; i++) {
                menu.addItem(new BMap.MenuItem(txtMenuItem[i].text, txtMenuItem[i].callback, 100));
                if (i >= 1 || i == 0) {
                    menu.addSeparator();  //Add right click menu to split line.
                }
            }
            map.addContextMenu(menu);  //Add menu to map
        },
        remove: function (marker) {
            map.clearOverlays(marker);
            markerVal.delete("start");
            markerVal.delete("end");
            $(".route-start input").val("");
            $(".route-end input").val("");
        },

        addLabel: function (data) {
            var lng = data["lng"];
            var lat = data["lat"];
            var msg = data["msg"];
            var BD09 = coordinate.WGS84ToBD09LL(lng, lat);
            lng = BD09[0];
            lat = BD09[1];
            var point = new BMap.Point(lng, lat);
            var opts = {
                position: point,
                offset: new BMap.Size(30, -30)
            }
            var label = new BMap.Label(msg, opts);
            label.setStyle({
                color: "red",
                fontSize: "12px",
                height: "20px",
                lineHeight: "20px",
                fontFamily: "微软雅黑"
            });
            map.addOverlay(label);
        },

        getMarkerVal: function () {
            return markerVal;
        },

        getBMap: function () {
            return map;
        },
        createMarker: function (data) {
            var data = JSON.parse(data.data);
            var latitude = data.autoDrivingCar.latitude;
            var longitude = data.autoDrivingCar.longitude;
            var BD09 = coordinate.WGS84ToBD09LL(longitude, latitude);
            var point = new BMap.Point(BD09[0], BD09[1]);
            var rotation;
            if (markerVal.get(1) != null) {
                var marker = markerVal.get(1);
                map.removeOverlay(marker);
                var marker = new BMap.Marker(point, { icon: myIcon });
                markerVal.set(1, marker);
                //map.centerAndZoom(point, 19);
                map.addOverlay(marker);
                marker.setPosition(point);
                if (oldPoint != undefined && oldPoint != "") {
                    getRotation(oldPoint, point, marker);
                }
            } else {
                var marker = new BMap.Marker(point, { icon: myIcon });
                markerVal.set(1, marker);
                //map.centerAndZoom(point, 19);
                map.addOverlay(marker);
            }
            oldPoint = point;
        },
        mapRendering: function (event) {
            var overlays = [];
            var data = event;
            var detailedData = data;
            var styleOptions = {
                strokeWeight: '3',
                strokeOpacity: 0.8,
                strokeColor: "#18a45b"
            };
            for (var i = 0; i < detailedData.length; i++) {
                var BD09 = coordinate.WGS84ToBD09LL(detailedData[i].lng, detailedData[i].lat);
                overlays.push(new BMap.Point(BD09[0], BD09[1]));
            }

            map.addOverlay(new BMap.Polyline(overlays, styleOptions));
            map.centerAndZoom(overlays[overlays.length - 1], 17);
        },
    };
    function getRotation(curPos, targetPos, mk) {
        var me = this;
        var deg = 0;
        var result;
        curPos = map.pointToPixel(curPos);
        targetPos = map.pointToPixel(targetPos);
        if (targetPos.x != curPos.x) {
            var tan = (targetPos.y - curPos.y) / (targetPos.x - curPos.x),
                atan = Math.atan(tan);
            deg = atan * 360 / (2 * Math.PI);
            if (targetPos.x < curPos.x) {
                deg = -deg + 90 + 90;
            } else {
                deg = -deg;
            }
            mk.setRotation(-deg);
            console.log(-deg);

        } else {
            var disy = targetPos.y - curPos.y;
            var bias = 0;
            if (disy > 0)
                bias = -1
            else
                bias = 1


            mk.setRotation(-bias * 90);
            console.log(-bias * 90);
        }
        return;
    }
    exports('map', obj);
});