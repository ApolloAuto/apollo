import { observable, action } from 'mobx';

import _ from 'lodash';

import RENDERER from 'renderer';
import MAP_NAVIGATOR from 'components/Navigation/MapNavigator';

export default class RouteEditingManager {
    // Map from POI name to its x,y coordinates,
    // e.g. {POI-1: [{x: 1.0, y: 1.2}, {x: 101.0, y: 10.2}]}
    @observable defaultRoutingEndPoint = {};

    @observable defaultRoutings = {};

    @observable currentPOI = 'none';

    @observable inDefaultRoutingMode = false;

    defaultParkingInfo = {};

    currentDefaultRouting = 'none';

    @action updateDefaultRoutingEndPoint(data) {
      if (data.poi === undefined) {
        return;
      }
      this.defaultRoutingEndPoint = {};
      this.defaultParkingInfo = {};
      for (let i = 0; i < data.poi.length; ++i) {
        const place = data.poi[i];
        this.defaultRoutingEndPoint[place.name] = place.waypoint;
        this.defaultParkingInfo[place.name] = place.parkingInfo;

        // Default string value is empty string in proto.
        // Remove this unset field here to prevent empty string
        // sends in routing request.
        if (this.defaultParkingInfo[place.name].parkingSpaceId === '') {
          delete this.defaultParkingInfo[place.name].parkingSpaceId;
          if (_.isEmpty(this.defaultParkingInfo[place.name])) {
            delete this.defaultParkingInfo[place.name];
          }
        }
      }
    }

    @action addDefaultEndPoint(poiName, inNavigationMode) {
      if (_.isEmpty(this.defaultRoutingEndPoint)) {
        alert("Failed to get default routing end point, make sure there's "
                + 'a default end point file under the map data directory.');
        return;
      }
      if (poiName === undefined || poiName === ''
            || !(poiName in this.defaultRoutingEndPoint)) {
        alert('Please select a valid POI.');
        return;
      }
      this.currentPOI = poiName;

      if (inNavigationMode) {
        MAP_NAVIGATOR.addDefaultEndPoint(this.defaultRoutingEndPoint[poiName]);
      } else {
        RENDERER.addDefaultEndPoint(this.defaultRoutingEndPoint[poiName]);
        RENDERER.setParkingInfo(this.defaultParkingInfo[poiName]);
      }
    }

    @action addDefaultRoutingPoint(defaultRoutingName) {
      if (_.isEmpty(this.defaultRoutings)) {
        alert("Failed to get default routing, make sure there's "
                + 'a default routing file under the map data directory.');
        return;
      }
      if (defaultRoutingName === undefined || defaultRoutingName === ''
            || !(defaultRoutingName in this.defaultRoutings)) {
        alert('Please select a valid default routing.');
        return;
      }

      RENDERER.addDefaultEndPoint(this.defaultRoutings[defaultRoutingName], false);
    }

    @action updateDefaultRoutingPoints(data) {
      if (data.defaultRoutings === undefined) {
        return;
      }
      this.defaultRoutings = {};
      for (let i = 0; i < data.defaultRoutings.length; ++i) {
        const drouting = data.defaultRoutings[i];
        this.defaultRoutings[drouting.name] = drouting.point;
      }
    }

    addDefaultRoutingPath(message) {
      if (message.data === undefined) {
        return;
      }
      const drouting = message.data;
      this.defaultRoutings[drouting.name] = drouting.point;
    }

    addDefaultRouting(routingName) {
      return RENDERER.addDefaultRouting(routingName);
    }

    toggleDefaultRoutingMode() {
      this.inDefaultRoutingMode = !this.inDefaultRoutingMode;
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
      }
      return MAP_NAVIGATOR.sendRoutingRequest();
    }

    sendCycleRoutingRequest(defaultRoutingName, cycleNumber) {
      const points = this.defaultRoutings[defaultRoutingName];
      if (!isNaN(cycleNumber) || !points) {
        const success = RENDERER.sendCycleRoutingRequest
        (defaultRoutingName, points, cycleNumber);
        if (success) {
          this.disableRouteEditing();
        }
        return success;
      }
      return false;
    }
}
