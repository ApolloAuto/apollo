import { observable, action } from "mobx";

import RENDERER from "renderer";
import MAP_NAVIGATOR from "components/Navigation/MapNavigator";

export default class RouteEditingManager {

    // Map from POI name to its x,y coordinates,
    // e.g. {POI-1: [{x: 1.0, y: 1.2}, {x: 101.0, y: 10.2}]}
    @observable defaultRoutingEndPoint = {};
    @observable defaultParkingSpaceId = {};
    @observable currentPOI = "none";


    @action updateDefaultRoutingEndPoint(data) {
        if (data.poi === undefined) {
            return;
        }
        this.defaultRoutingEndPoint = {};
        this.defaultParkingSpaceId = {};
        for (let i = 0; i < data.poi.length; ++i) {
            const place = data.poi[i];
            this.defaultRoutingEndPoint[place.name] = place.waypoint;
            this.defaultParkingSpaceId[place.name] = place.parkingSpaceId;
        }
    }

    @action addDefaultEndPoint(poiName, inNavigationMode) {
        if (_.isEmpty(this.defaultRoutingEndPoint)) {
            alert("Failed to get default routing end point, make sure there's " +
                  "a default end point file under the map data directory.");
            return;
        }
        if (poiName === undefined || poiName === ""
            || !(poiName in this.defaultRoutingEndPoint)) {
            alert("Please select a valid POI.");
            return;
        }
        this.currentPOI = poiName;

        if (inNavigationMode) {
            MAP_NAVIGATOR.addDefaultEndPoint(this.defaultRoutingEndPoint[poiName]);
        } else {
            RENDERER.addDefaultEndPoint(this.defaultRoutingEndPoint[poiName]);
            RENDERER.setParkingSpaceId(this.defaultParkingSpaceId[poiName]);
        }
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

    sendRoutingRequest(inNavigationMode) {
        if (!inNavigationMode) {
            const success = RENDERER.sendRoutingRequest();
            if (success) {
                this.disableRouteEditing();
            }
            return success;
        } else {
            return MAP_NAVIGATOR.sendRoutingRequest();
        }
    }
}
