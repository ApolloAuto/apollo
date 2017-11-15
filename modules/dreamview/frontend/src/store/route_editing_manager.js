import { observable, action } from "mobx";

import RENDERER from "renderer";

export default class RouteEditingManager {

    // Map from POI name to its x,y coordinates, e.g. {POI-1: {x: 1.0, y: 1.2}}
    @observable defaultRoutingEndPoint = {};
    @observable currentPOI = "none"; // The chosen POI as a routing end point.

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

    @action setDefaultEndPoint(poiName) {
        if (_.isEmpty(this.defaultRoutingEndPoint)) {
            alert("Failed to get default routing end point, make sure there's " +
                  "a default end point file under the map data directory.");
            return;
        }
        if (poiName === undefined || poiName === "none"
            || !(poiName in this.defaultRoutingEndPoint)) {
            alert("Please select a valid point of interest.");
            return;
        }
        this.currentPOI = poiName;
    }

    addDefaultEndPoint() {
        if (this.currentPOI === "none") {
            alert("Please select a valid point of interest.");
            return false;
        }
        RENDERER.addDefaultEndPoint(this.defaultRoutingEndPoint[this.currentPOI]);
        return true;
    }

    enableRouteEditing() {
        RENDERER.enableRouteEditing();
    }

    disableRouteEditing() {
        RENDERER.disableRouteEditing();
    }

    removeLastRoutingPoint() {
        RENDERER.removeLastRoutingPoint();
    }

    removeAllRoutingPoints() {
        RENDERER.removeAllRoutingPoints();
    }

    sendRoutingRequest() {
        const success = RENDERER.sendRoutingRequest();
        if (success) {
            this.disableRouteEditing();
        }
        return success;
    }
}
