import { observable, action } from "mobx";

import RENDERER from "renderer";

export default class RouteEditingManager {

    @observable inEditingView = false;
    // Map from POI name to its x,y coordinates, e.g. {POI-1: {x: 1.0, y: 1.2}}
    @observable defaultRoutingEndPoint = {};
    @observable poi_name = ""; // The chosen POI as a routing end point.

    @action updateDefaultRoutingEndPoint(data) {
        if (data.poi === undefined || _.isEmpty(data.poi)) {
            return;
        }
        this.defaultRoutingEndPoint = {};
        for (let i = 0; i < data.poi.length; ++i) {
            const place = data.poi[i];
            this.defaultRoutingEndPoint[place.name] = {
                    x: place.x,
                    y: place.y
                };
        }
    }

    @action addDefaultEndPoint(poi_name) {
        this.poi_name = poi_name;
        if (_.isEmpty(this.defaultRoutingEndPoint)) {
            alert("Failed to get default routing end point, make sure there's " +
                  "a default end point file under the map data directory.");
            return;
        }
        if (poi_name === undefined || poi_name === ""
            || !(poi_name in this.defaultRoutingEndPoint)) {
            alert("Please select a valid POI.");
            return;
        }
        RENDERER.addDefaultEndPoint(this.defaultRoutingEndPoint[poi_name]);
    }

    @action enableRouteEditing() {
        this.inEditingView = true;
        RENDERER.enableRouteEditing();
    }

    @action disableRouteEditing() {
        this.inEditingView = false;
        RENDERER.disableRouteEditing();
    }

    removeLastRoutingPoint() {
        RENDERER.removeLastRoutingPoint();
    }

    removeAllRoutingPoints() {
        RENDERER.removeAllRoutingPoints();
    }

    sendRoutingRequest() {
        if (RENDERER.sendRoutingRequest()){
            this.disableRouteEditing();
        }
    }
}
