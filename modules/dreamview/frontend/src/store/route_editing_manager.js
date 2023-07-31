import { observable, action } from 'mobx';

import _ from 'lodash';

import RENDERER from 'renderer';
import MAP_NAVIGATOR from 'components/Navigation/MapNavigator';

export default class RouteEditingManager {
    // Map from POI name to its x,y coordinates,
    // e.g. {POI-1: [{x: 1.0, y: 1.2}, {x: 101.0, y: 10.2}]}
    @observable defaultRoutingEndPoint = {};

    @observable defaultRoutings = {};

    @observable defaultRoutingDistanceThreshold = 10.0;

    @observable currentPOI = 'none';

    @observable inDefaultRoutingMode = false;

    defaultParkingInfo = {};

    currentDefaultRouting = 'none';

    parkingRoutingDistanceThreshold = 20.0;

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
      const routings = this.defaultRoutings;
      if (_.isEmpty(routings)) {
        alert('Failed to get routing, make sure the '
                + 'routing file under the map data directory.');
        return;
      }
      if (!defaultRoutingName || !(defaultRoutingName in routings)) {
        alert('Please select a valid default routing.');
        return;
      }

      RENDERER.addDefaultEndPoint(routings[defaultRoutingName]);
    }

    @action updateDefaultRoutingPoints(data) {
      if (data.threshold) {
        this.defaultRoutingDistanceThreshold = data.threshold;
      }
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
      const waypoints = drouting.waypoint.map(
        point => _.assign({}, point.pose, { heading: point.heading })
      );
      this.defaultRoutings[drouting.name] = waypoints;
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

    setStartPoint() {
      return RENDERER.setStartPoint();
    }

    sendRoutingRequest(inNavigationMode, defaultRoutingName = '') {
      if (!inNavigationMode) {
        const success = _.isEmpty(defaultRoutingName) ? RENDERER.sendRoutingRequest()
          : RENDERER.sendRoutingRequest(this.defaultRoutings[defaultRoutingName]);
        if (success) {
          this.disableRouteEditing();
        }
        return success;
      }
      return MAP_NAVIGATOR.sendRoutingRequest();
    }

    sendCycleRoutingRequest(cycleNumber) {
      const points = this.defaultRoutings[this.currentDefaultRouting];
      if (!isNaN(cycleNumber) || !points) {
        const success = RENDERER.sendCycleRoutingRequest
        (this.currentDefaultRouting, points, cycleNumber);
        if (success) {
          this.disableRouteEditing();
        }
        return success;
      }
      return false;
    }

    checkCycleRoutingAvailable() {
      return RENDERER.checkCycleRoutingAvailable(this.defaultRoutings[this.currentDefaultRouting],
        this.defaultRoutingDistanceThreshold);
    }

    updateParkingRoutingDistance(data) {
      if (data.threshold) {
        this.parkingRoutingDistanceThreshold = data.threshold;
      }
    }
}
