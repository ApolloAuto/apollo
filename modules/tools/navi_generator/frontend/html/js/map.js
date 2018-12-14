
//Map leading line drawing
layui.define(['jquery','layer','upload','coordinate'],function(exports){
    var $ = layui.jquery,
    layer = layui.layer,
    upload = layui.upload,
    coordinate = layui.coordinate;
    var map = new BMap.Map('allmap');
    var markerVal=new Map();
    var startmyIcon = new BMap.Icon("images/start.png", new BMap.Size(32,43));
    var waypointmyIcon = new BMap.Icon("images/pathPoint.png", new BMap.Size(32,43));
    var endmyIcon = new BMap.Icon("images/end.png", new BMap.Size(32,43));
    var obj = {
        map:function (){
            var point = new BMap.Point(112.885542, 28.122144);
            var geoc = new BMap.Geocoder();
            map.centerAndZoom(point, 16);
            map.addControl(new BMap.OverviewMapControl());       //Adding abbreviated map
            map.enableScrollWheelZoom();                         //Enable rollers to zoom in and out
            map.addControl(new BMap.NavigationControl());
            map.disable3DBuilding();
            changeMapStyle('dark')  //Custom map color
            function changeMapStyle(style){
              map.setMapStyle({style:style});
            }
            //Create right menu
            var menu = new BMap.ContextMenu();
            var txtMenuItem = [
                {
                    text:'Starting point',
                    callback:function(e){
                        if(markerVal.get("start")){
                            map.removeOverlay(markerVal.get("start"));
                        }
                        var pt=new BMap.Point(parseFloat(e.lng), parseFloat(e.lat));
                        var marker = new BMap.Marker(pt,{icon:startmyIcon});  // Create annotation
                        marker.enableDragging();
                        markerVal.set("start",marker);
                        map.addOverlay(marker);
                        geoc.getLocation(pt, function(rs){
                            var addComp = rs.addressComponents;
                            var site = addComp.province + addComp.city+ addComp.district + addComp.street + addComp.streetNumber;
                            $(".route-start input").val(site);
                        });
                    }
                },
                {
                    text:'End point',
                    callback:function(e){
                        if(markerVal.get("end")){
                            map.removeOverlay(markerVal.get("end"));
                        }
                        var pt=new BMap.Point(parseFloat(e.lng), parseFloat(e.lat));
                        var marker3 = new BMap.Marker(pt,{icon:endmyIcon});
                        markerVal.set("end",marker3);
                        marker3.enableDragging();
                        map.addOverlay(marker3);
                        geoc.getLocation(pt, function(rs){
                            var addComp = rs.addressComponents;
                            var site = addComp.province + addComp.city+ addComp.district + addComp.street + addComp.streetNumber;
                            $(".route-end input").val(site)
                        });
                    }
                }
            ];
            for(var i=0; i < txtMenuItem.length; i++){
                menu.addItem(new BMap.MenuItem(txtMenuItem[i].text,txtMenuItem[i].callback,100));
                if(i >= 1 || i == 0) {
                    menu.addSeparator();  //Add right click menu to split line.
                }
            }
            map.addContextMenu(menu);  //Add menu to map
      },
      remove:function(marker){
        map.clearOverlays(marker);
        markerVal.delete("start");
        markerVal.delete("end");
        $(".route-start input").val("");
        $(".route-end input").val("");
      },

      addLabel:function(data){
        var lng = data["Lng"];
        var lat = data["Lat"];
        var msg = data["msg"];
        var BD09 = coordinate.WGS84ToBD09LL(lng,lat);
        lng = BD09[0];
        lat = BD09[1];
        var point = new BMap.Point(lng,lat);
        var opts = {
            position : point,
            offset   : new BMap.Size(30, -30)
        }
        var label = new BMap.Label(msg, opts);
        label.setStyle({
            color : "red",
            fontSize : "12px",
            height : "20px",
            lineHeight : "20px",
            fontFamily:"微软雅黑"
        });
        map.addOverlay(label);
      },

        getMarkerVal:function(){
            return markerVal;
        },

        getBMap:function(){
            return map;
        },

        mapRendering:function(event){
         var overlays=[];
         var data = event;
         var detailedData = data;
         var styleOptions = {
                    strokeColor:"green",//Setting color
                    strokeWeight:6,
                    strokeOpacity:1
                };
        for(var i=0; i<detailedData.length; i++){
            var BD09 = coordinate.WGS84ToBD09LL(detailedData[i].Lng,detailedData[i].Lat);
            overlays.push(new BMap.Point(BD09[0],BD09[1]));
        }

        map.addOverlay(new BMap.Polyline(overlays, styleOptions));
        map.centerAndZoom(overlays[overlays.length-1], 17);
      }
    };
    exports('map', obj);
});