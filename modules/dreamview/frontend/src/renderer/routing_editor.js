import 'imports-loader?THREE=three!three/examples/js/controls/OrbitControls.js';

import routingPointPin from 'assets/images/routing/pin.png';

import WS from 'store/websocket';
import { drawImage } from 'utils/draw';

const minDefaultRoutingPointsNum = 1;
const maxDistance = 5;

export default class RoutingEditor {
  constructor() {
    this.routePoints = [];
    this.parkingInfo = null;
    this.inEditingMode = false;
    this.pointId = 0;
  }

  isInEditingMode() {
    return this.inEditingMode;
  }

  enableEditingMode(camera, adc) {
    this.inEditingMode = true;

    const pov = 'Map';
    camera.fov = PARAMETERS.camera[pov].fov;
    camera.near = PARAMETERS.camera[pov].near;
    camera.far = PARAMETERS.camera[pov].far;

    camera.updateProjectionMatrix();
    WS.requestMapElementIdsByRadius(PARAMETERS.routingEditor.radiusOfMapRequest);
  }

  disableEditingMode(scene) {
    this.inEditingMode = false;
    this.removeAllRoutePoints(scene);
    this.parkingInfo = null;
    this.pointId = 0;
  }

  addRoutingPoint(point, coordinates, scene, offset = true) {
    const offsetPoint = offset ? coordinates.applyOffset({ x: point.x, y: point.y }) : point;
    const pointMesh = drawImage(routingPointPin, 3.5, 3.5, offsetPoint.x, offsetPoint.y, 0.3);
    pointMesh.pointId = this.pointId;
    point.id = this.pointId;
    this.pointId += 1;
    this.routePoints.push(pointMesh);
    scene.add(pointMesh);
    if (offset) {
      // Default routing has been checked
      WS.checkRoutingPoint(point);
    }
  }

  setParkingInfo(info) {
    this.parkingInfo = info;
  }

  removeInvalidRoutingPoint(pointId, msg, scene) {
    alert(msg);
    if (pointId) {
      this.routePoints = this.routePoints.filter((point) => {
        if (point.pointId === pointId) {
          this.removeRoutingPoint(scene, point);
          return false;
        }
        return true;
      });
    }
  }

  removeLastRoutingPoint(scene) {
    const lastPoint = this.routePoints.pop();
    if (lastPoint) {
      this.removeRoutingPoint(scene, lastPoint);
    }
  }

  removeAllRoutePoints(scene) {
    this.routePoints.forEach((object) => {
      this.removeRoutingPoint(scene, object);
    });
    this.routePoints = [];
  }

  removeRoutingPoint(scene, object) {
    scene.remove(object);
    if (object.geometry) {
      object.geometry.dispose();
    }
    if (object.material) {
      object.material.dispose();
    }
  }

  sendRoutingRequest(carOffsetPosition, carHeading, coordinates) {
    if (this.routePoints.length === 0) {
      alert('Please provide at least an end point.');
      return false;
    }

    const points = this.routePoints.map((object) => {
      object.position.z = 0;
      return coordinates.applyOffset(object.position, true);
    });
    const start = (points.length > 1) ? points[0]
      : coordinates.applyOffset(carOffsetPosition, true);
    const start_heading = (points.length > 1) ? null : carHeading;
    const end = points[points.length - 1];
    const waypoint = (points.length > 1) ? points.slice(1, -1) : [];
    WS.requestRoute(start, start_heading, waypoint, end, this.parkingInfo);

    return true;
  }

  sendCycleRoutingRequest(routingName, cycleRoutingPoints, cycleNumber,
    carOffsetPosition, carHeading, coordinates) {
    const points = cycleRoutingPoints.map((point) => {
      point.z = 0;
      return coordinates.applyOffset(point, true);
    });
    const start = coordinates.applyOffset(carOffsetPosition, true);
    const start_heading = carHeading;
    const end = points[points.length - 1];
    const waypoint = (points.length > 1) ? points.slice(0, -1) : [];
    WS.requestDefaultCycleRouting(start, start_heading, waypoint, end, cycleNumber);
    return true;
  }

  addDefaultRouting(routingName) {
    if (this.routePoints.length < minDefaultRoutingPointsNum) {
      alert(`Please provide at least ${minDefaultRoutingPointsNum} end point.`);
      return false;
    }

    const points = this.routePoints.map((object) => {
      return object.position;
    });
    if (!this.checkDefaultRoutingAvailable(points[0], points[points.length - 1])) {
      alert(`Please set the default routing reasonably,the distance from the start point to the end 
point should not exceed ${maxDistance},otherwise it will not be able to form a closed loop.`);
      return false;
    }
    WS.saveDefaultRouting(routingName, points);
  }

  checkDefaultRoutingAvailable(start, end) {
    if (_.isEmpty(start) || _.isEmpty(end)) {
      return false;
    }
    const distance = Math.sqrt(Math.pow((end.x - start.x), 2) + Math.pow((end.y - start.y), 2));
    return distance <= maxDistance;
  }
}
