import 'imports-loader?THREE=three!three/examples/js/controls/OrbitControls.js';

import routingPointPin from 'assets/images/routing/pin.png';

import WS from 'store/websocket';
import { drawImage } from 'utils/draw';
import { IsPointInRectangle } from 'utils/misc';
import _ from 'lodash';

const minDefaultRoutingPointsNum = 1;

export default class RoutingEditor {
  constructor() {
    this.routePoints = [];
    this.parkingInfo = null;
    this.inEditingMode = false;
    this.pointId = 0;
    this.parkingSpaceInfo = [];
    this.deadJunctionInfo = [];
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

  addRoutingPoint(point, coordinates, scene) {
    const offsetPoint = coordinates.applyOffset({ x: point.x, y: point.y });
    const selectedParkingSpaceIndex = this.isPointInParkingSpace(offsetPoint);
    if (selectedParkingSpaceIndex !== -1) {
      this.parkingSpaceInfo[selectedParkingSpaceIndex].selectedCounts++;
    }
    const pointMesh = drawImage(routingPointPin, 3.5, 3.5, offsetPoint.x, offsetPoint.y, 0.3);
    pointMesh.pointId = this.pointId;
    point.id = this.pointId;
    this.pointId += 1;
    this.routePoints.push(pointMesh);
    scene.add(pointMesh);
    WS.checkRoutingPoint(point);
    return selectedParkingSpaceIndex;
  }

  setParkingInfo(info) {
    this.parkingInfo = info;
  }

  setParkingSpaceInfo(parkingSpaceInfo, extraInformation, coordinates, scene) {
    this.parkingSpaceInfo = parkingSpaceInfo;
    this.parkingSpaceInfo.forEach((item, index) => {
      const extraInfo = extraInformation[index];
      if (_.isEmpty(extraInfo)) {
        return false;
      }
      const offsetPoints = item.polygon.point.map(point =>
        coordinates.applyOffset({ x: point.x, y: point.y })
      );
      const adjustPoints = extraInfo.order.map(number => {
        return offsetPoints[number];
      });
      item.polygon.point = adjustPoints;
      item.type = extraInfo.type;
      item.routingEndPoint = extraInfo.routingEndPoint;
      item.laneWidth = extraInfo.laneWidth;
      item.selectedCounts = 0;
      item.laneId = extraInfo.laneId;
    });
  }

  removeInvalidRoutingPoint(pointId, msg, scene) {
    let index = -1;
    alert(msg);
    if (pointId) {
      this.routePoints = this.routePoints.filter((point) => {
        if (point.pointId === pointId) {
          index = this.removeRoutingPoint(scene, point);
          return false;
        }
        return true;
      });
    }
    return index;
  }

  removeLastRoutingPoint(scene) {
    const lastPoint = this.routePoints.pop();
    let index = -1;
    if (lastPoint) {
      index = this.removeRoutingPoint(scene, lastPoint);
    }
    return index;
  }

  removeAllRoutePoints(scene) {
    let index = -1;
    const indexArr = [];
    this.routePoints.forEach((object) => {
      index = this.removeRoutingPoint(scene, object);
      if (index !== -1) {
        indexArr.push(index);
      }
    });
    this.routePoints = [];
    return indexArr;
  }

  removeRoutingPoint(scene, object) {
    scene.remove(object);
    let index = this.isPointInParkingSpace(_.get(object, 'position'));
    if (index !== -1) {
      if (--this.parkingSpaceInfo[index].selectedCounts > 0) {
        index = -1;
      }
    }
    if (object.geometry) {
      object.geometry.dispose();
    }
    if (object.material) {
      object.material.dispose();
    }
    return index;
  }

  sendRoutingRequest(carOffsetPosition, carHeading, coordinates, routingPoints) {
    // parking routing request vs common routing request
    // add dead end junction routing request when select three points
    // and the second point is in dead end junction.
    if (this.routePoints.length === 0 && routingPoints.length === 0) {
      alert('Please provide at least an end point.');
      return false;
    }

    const index = _.isEmpty(this.routePoints) ?
      -1 : this.isPointInParkingSpace(this.routePoints[this.routePoints.length - 1].position);
    const parkingRoutingRequest = (index !== -1);
    if (parkingRoutingRequest) {
      const lastPoint = this.routePoints.pop();
      const { routingEndPoint, id, type, laneWidth, laneId } = this.parkingSpaceInfo[index];
      const parkingRequestPoints = this.routePoints.map((object) => {
        object.position.z = 0;
        return coordinates.applyOffset(object.position, true);
      });
      parkingRequestPoints.push(coordinates.applyOffset(routingEndPoint, true));
      const start = (parkingRequestPoints.length > 1) ? parkingRequestPoints[0]
        : coordinates.applyOffset(carOffsetPosition, true);
      const end = parkingRequestPoints[parkingRequestPoints.length - 1];
      const waypoint = (parkingRequestPoints.length > 1) ? parkingRequestPoints.slice(1, -1) : [];
      this.routePoints.push(lastPoint);
      this.parkingSpaceInfo[index].polygon.point.forEach(vertex => {
        vertex.z = 0;
        parkingRequestPoints.push(coordinates.applyOffset(vertex, true));
      });
      const cornerPoints = parkingRequestPoints.slice(-4);
      const parkingInfo = {
        parkingSpaceId: _.get(id, 'id'),
        parkingPoint: coordinates.applyOffset(lastPoint.position, true),
        parkingSpaceType: type,
      };
      WS.sendParkingRequest(start, waypoint, end, parkingInfo, laneWidth, cornerPoints, laneId);
      return true;
    }
    const points = _.isEmpty(routingPoints) ?
      this.routePoints.map((object) => {
        object.position.z = 0;
        return coordinates.applyOffset(object.position, true);
      }) : routingPoints.map((point) => {
        point.z = 0;
        return _.pick(point, ['x', 'y', 'z']);
      });
    if (points.length === 3 && !_.isEmpty(this.deadJunctionInfo)) {
      const deadJunctionIdx = this.determinDeadEndJunctionRequest(points);
      if (deadJunctionIdx !== -1) {
        const deadJunction = this.deadJunctionInfo[deadJunctionIdx];
        WS.sendDeadEndJunctionRoutingRequest(
          points[0], deadJunction.inEndPoint, deadJunction.outStartPoint,
          points[2], deadJunction.inLaneIds, deadJunction.outLaneIds,
          [deadJunction.routingPoint],
        );
        return true;
      }
    }
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
      return _.pick(point, ['x', 'y', 'z']);
    });
    const start = coordinates.applyOffset(carOffsetPosition, true);
    const start_heading = carHeading;
    const end = points[points.length - 1];
    const waypoint = (points.length > 1) ? points.slice(0, -1) : [];
    WS.requestDefaultCycleRouting(start, start_heading, waypoint, end, cycleNumber);
    return true;
  }

  addDefaultRouting(routingName, coordinates) {
    if (this.routePoints.length < minDefaultRoutingPointsNum) {
      alert(`Please provide at least ${minDefaultRoutingPointsNum} end point.`);
      return false;
    }

    const points = this.routePoints.map(object =>
      coordinates.applyOffset(object.position, true)
      // the coordinates of default routing are consistent with poi
    );
    WS.saveDefaultRouting(routingName, points);
  }

  checkCycleRoutingAvailable(cycleRoutingPoints, carPosition, threshold) {
    const start = carPosition;
    const end = cycleRoutingPoints[cycleRoutingPoints.length - 1];
    if (_.isEmpty(start) || _.isEmpty(end)) {
      return false;
    }
    const distance =
      Math.sqrt(Math.pow((end.x - start.x), 2) + Math.pow((end.y - start.y), 2));
    return distance > threshold;
  }

  isPointInParkingSpace(offsetPoint) {
    let index = -1;
    if (!_.isEmpty(this.parkingSpaceInfo) && !_.isEmpty(offsetPoint)) {
      index = _.findIndex(this.parkingSpaceInfo, item =>
        IsPointInRectangle(item.polygon.point, offsetPoint));
    }
    return index;
  }

  determinDeadEndJunctionRequest(offsetPoints) {
    const index = _.findIndex(this.deadJunctionInfo, deadJunction =>
      IsPointInRectangle(deadJunction.deadJunctionPoints, offsetPoints[1]));
    return index;
  }

  setDeadJunctionInfo(deadJunctionInfos) {
    this.deadJunctionInfo = deadJunctionInfos;
  }
}
