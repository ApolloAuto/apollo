layui.define(['jquery','chncrs'],function(exports){
    var $ = layui.jquery,
    chncrs = layui.chncrs;
    var converter = {
         lngLatToJson:function (latitude, longitude) {
            var json = {"Lat": latitude, "Lng": longitude};
            return json;
        },
        BD09LLToGCJ02:function (longitude, latitude) {
            return chncrs.transform([longitude, latitude], "BD09LL", "GCJ02");
        },

        BD09LLToWGS84:function (longitude, latitude) {
            if (outOfChina(longitude, latitude)) {
                return [longitude, latitude];
            } else {
                const GCJ02 = converter.BD09LLToGCJ02(longitude, latitude);
                return chncrs.transform(GCJ02, "GCJ02", "WGS84");
            }
        },

        WGS84ToBD09LL:function (longitude, latitude) {
            if (outOfChina(longitude, latitude)) {
                return [longitude, latitude];
            } else {
                var GCJ02 = converter.WGS84ToGCJ02(longitude, latitude);
                return chncrs.transform(GCJ02, "GCJ02", "BD09LL");
            }
        },
        WGS84ToGCJ02:function (longitude, latitude) {
            return chncrs.transform([longitude, latitude], "WGS84", "GCJ02");
        },
        baiduApiToJson:function (start, end,driving,type) {
            var startWGS84 = converter.BD09LLToWGS84(start.point.lng, start.point.lat);
            var startWGS84Json = converter.lngLatToJson(startWGS84[1], startWGS84[0]);
            var endWGS84 = converter.BD09LLToWGS84(end.point.lng, end.point.lat);
            var endWGS84Json = converter.lngLatToJson(endWGS84[1], endWGS84[0]);
            var result = {"id": "requestRoute"};

            var param = {
                "start": startWGS84Json,    // the starting point in wgs84
                "end": endWGS84Json,    // the end point in wgs84
                //"waypoint": waypointArray,
                "type":type
            };
            param.numPlans = driving.getResults().getNumPlans();                            // the number of plans
            var routePlans = new Array();
            for (var i = 0; i < driving.getResults().getNumPlans(); i++) {
                var routePlansJson = {};
                routePlansJson.routePlanIndex = i;
                var numRoutes = driving.getResults().getPlan(i).getNumRoutes();
                for (var j = 0; j < numRoutes; j++) {
                    var routePlan = {};
                    routePlan.numRoutes = numRoutes;
                    var routes = new Array();
                    var routesJson = {};
                    routesJson.routeIndex = j;
                    var route = {};
                    var step = new Array();
                    route.numSteps = driving.getResults().getPlan(i).getRoute(j).getNumSteps();
                    for (var k = 0; k < route.numSteps; k++) {
                        var stepLng = driving.getResults().getPlan(i).getRoute(j).getStep(k).getPosition().lng;
                        var steplat = driving.getResults().getPlan(i).getRoute(j).getStep(k).getPosition().lat;
                        var numStepsWGS84 = converter.BD09LLToWGS84(stepLng, steplat);
                        var numStepsWGS84Json = converter.lngLatToJson(numStepsWGS84[1], numStepsWGS84[0]);
                        numStepsWGS84Json.stepIndex = k;
                        step.push(numStepsWGS84Json);
                    }
                    var pathArray = driving.getResults().getPlan(i).getRoute(j).getPath();
                    var path = new Array();
                    for (var h = 0; h < pathArray.length; h++) {
                        var pathLng = pathArray[h].lng;
                        var pathlat = pathArray[h].lat;
                        var pathWGS84 = converter.BD09LLToWGS84(pathLng, pathlat);
                        var pathWGS84Json = converter.lngLatToJson(pathWGS84[1], pathWGS84[0]);
                        path.push(pathWGS84Json);
                    }
                    route.step = step;
                    route.path = path;
                    routesJson.route = route;
                    routes.push(routesJson);
                    routePlan.routes = routes;
                    routePlansJson.routePlan = routePlan;
                    routePlans.push(routePlansJson);
                }
                param.routePlans = routePlans;
                result.param=param;
                return param;
            }
        }
    };
    function outOfChina(longitude, latitude) {
            if (longitude < 72.004 || longitude > 137.8347) {
                return true;
            }
            if (latitude < 0.8293 || latitude > 55.8271) {
                return true;
            }
            return false;
        }
    exports('coordinate', converter);
});
