import * as THREE from 'three';
import STORE from 'store';
import { MAP_WS } from 'store/websocket';
import _ from 'lodash';

import {
  drawSegmentsFromPoints,
  drawDashedLineFromPoints,
  drawShapeFromPoints,
  changeMaterial,
} from 'utils/draw';
import Text3D, { TEXT_ALIGN } from 'renderer/text3d';
import TrafficSigns from 'renderer/traffic_controls/traffic_signs';
import TrafficSignals from 'renderer/traffic_controls/traffic_signals';

import stopSignMaterial from 'assets/models/stop_sign.mtl';
import stopSignObject from 'assets/models/stop_sign.obj';
import yieldSignMaterial from 'assets/models/yield_sign.mtl';
import yieldSignObject from 'assets/models/yield_sign.obj';

const STOP_SIGN_SCALE = 0.01;
const YIELD_SIGN_SCALE = 1.5;

const colorMapping = {
  YELLOW: 0XDAA520,
  WHITE: 0xCCCCCC,
  CORAL: 0xFF7F50,
  RED: 0xFF6666,
  GREEN: 0x006400,
  BLUE: 0x30A5FF,
  PURE_WHITE: 0xFFFFFF,
  DEFAULT: 0xC0C0C0,
};

// parallel=1 0-1 2-3
const order1 = [
  [0, 1, 2, 3],
  [1, 0, 3, 2],
  [2, 3, 0, 1],
  [3, 2, 1, 0],
];
// parallel=2 0-3 1-2
const order2 = [
  [0, 3, 2, 1],
  [1, 2, 3, 0],
  [2, 1, 0, 3],
  [3, 0, 1, 2],
];
export default class Map {
  constructor() {
    this.textRender = new Text3D();
    this.hash = -1;
    this.data = {};
    this.initialized = false;
    this.elementKindsDrawn = '';

    this.trafficSignals = new TrafficSignals();
    this.stopSigns = new TrafficSigns(
      stopSignMaterial, stopSignObject, STOP_SIGN_SCALE,
    );
    this.yieldSigns = new TrafficSigns(
      yieldSignMaterial, yieldSignObject, YIELD_SIGN_SCALE,
    );

    this.zOffsetFactor = 1;
  }

  // The result will be the all the elements in current but not in data.
  diffMapElements(elementIds, data) {
    const result = {};
    let empty = true;

    for (const kind in elementIds) {
      if (!this.shouldDrawObjectOfThisElementKind(kind)) {
        continue;
      }

      result[kind] = [];
      const newIds = elementIds[kind];
      const oldData = data[kind];
      for (let i = 0; i < newIds.length; ++i) {
        const found = oldData ? oldData.find((old) => old.id.id === newIds[i]) : false;

        if (!found) {
          empty = false;
          result[kind].push(newIds[i]);
        }
      }
    }

    return empty ? {} : result;
  }

  addLaneMesh(laneType, points) {
    switch (laneType) {
      case 'DOTTED_YELLOW':
        return drawDashedLineFromPoints(
          points, colorMapping.YELLOW, 4, 3, 3, this.zOffsetFactor, 1, false,
        );
      case 'DOTTED_WHITE':
        return drawDashedLineFromPoints(
          points, colorMapping.WHITE, 2, 0.5, 0.25, this.zOffsetFactor, 0.4, false,
        );
      case 'SOLID_YELLOW':
        return drawSegmentsFromPoints(
          points, colorMapping.YELLOW, 3, this.zOffsetFactor, false,
        );
      case 'SOLID_WHITE':
        return drawSegmentsFromPoints(
          points, colorMapping.WHITE, 3, this.zOffsetFactor, false,
        );
      case 'DOUBLE_YELLOW':
        const left = drawSegmentsFromPoints(
          points, colorMapping.YELLOW, 2, this.zOffsetFactor, false,
        );
        const right = drawSegmentsFromPoints(
          points.map((point) => new THREE.Vector3(point.x + 0.3, point.y + 0.3, point.z)),
          colorMapping.YELLOW, 3, this.zOffsetFactor, false,
        );
        left.add(right);
        return left;
      case 'CURB':
        return drawSegmentsFromPoints(
          points, colorMapping.CORAL, 3, this.zOffsetFactor, false,
        );
      default:
        return drawSegmentsFromPoints(
          points, colorMapping.DEFAULT, 3, this.zOffsetFactor, false,
        );
    }
  }

  addLane(lane, coordinates, scene) {
    const drewObjects = [];

    const centralLine = lane.centralCurve.segment;
    centralLine.forEach((segment) => {
      const points = coordinates.applyOffsetToArray(segment.lineSegment.point);
      const centerLine = drawSegmentsFromPoints(
        points, colorMapping.GREEN, 1, this.zOffsetFactor, false);
      centerLine.name = `CentralLine-${lane.id.id}`;
      scene.add(centerLine);
      drewObjects.push(centerLine);
    });

    const rightLaneType = lane.rightBoundary.boundaryType[0].types[0];
    // TODO: this is a temp. fix for repeated boundary types.
    lane.rightBoundary.curve.segment.forEach((segment, index) => {
      const points = coordinates.applyOffsetToArray(segment.lineSegment.point);
      const boundary = this.addLaneMesh(rightLaneType, points);
      boundary.name = `RightBoundary-${lane.id.id}`;
      scene.add(boundary);
      drewObjects.push(boundary);
    });

    const leftLaneType = lane.leftBoundary.boundaryType[0].types[0];
    lane.leftBoundary.curve.segment.forEach((segment, index) => {
      const points = coordinates.applyOffsetToArray(segment.lineSegment.point);
      const boundary = this.addLaneMesh(leftLaneType, points);
      boundary.name = `LeftBoundary-${lane.id.id}`;
      scene.add(boundary);
      drewObjects.push(boundary);
    });

    return drewObjects;
  }

  addLaneId(lane, coordinates, scene) {
    const centralLine = lane.centralCurve.segment;
    let position = _.get(centralLine, '[0].startPosition');
    if (position) {
      position.z = 0.04;
      position = coordinates.applyOffset(position);
    }

    const rotation = { x: 0.0, y: 0.0, z: 0.0 };
    const points = _.get(centralLine, '[0].lineSegment.point', []);
    if (points.length >= 2) {
      const p1 = points[0];
      const p2 = points[1];
      rotation.z = Math.atan2(p2.y - p1.y, p2.x - p1.x);
    }

    const text = this.textRender.drawText(
      lane.id.id, scene, colorMapping.WHITE, TEXT_ALIGN.LEFT,
    );
    if (text) {
      const textPosition = position || _.get(points, '[0]');
      if (textPosition) {
        text.position.set(textPosition.x, textPosition.y, textPosition.z);
        text.rotation.set(rotation.x, rotation.y, rotation.z);
      }
      text.visible = false;
      scene.add(text);
    }

    return text;
  }

  addRoad(road, coordinates, scene) {
    const drewObjects = [];

    road.section.forEach((section) => {
      section.boundary.outerPolygon.edge.forEach((edge) => {
        edge.curve.segment.forEach((segment, index) => {
          const points = coordinates.applyOffsetToArray(segment.lineSegment.point);
          const boundary = this.addLaneMesh('CURB', points);
          boundary.name = `Road-${road.id.id}`;
          scene.add(boundary);
          drewObjects.push(boundary);
        });
      });
    });

    return drewObjects;
  }

  addBorder(borderPolygon, color, coordinates, scene) {
    const drewObjects = [];

    const border = coordinates.applyOffsetToArray(borderPolygon.polygon.point);
    border.push(border[0]);

    const mesh = drawSegmentsFromPoints(
      border, color, 2, this.zOffsetFactor, true, false, 1.0,
    );
    scene.add(mesh);
    drewObjects.push(mesh);

    return drewObjects;
  }

  addParkingSpaceId(parkingSpace, coordinates, scene) {
    const text = this.textRender.drawText(parkingSpace.id.id, scene, colorMapping.WHITE);
    const points = _.get(parkingSpace, 'polygon.point');
    if (points && points.length >= 3 && text) {
      const point1 = points[0];
      const point2 = points[1];
      const point3 = points[2];
      let textPosition = {
        x: (point1.x + point3.x) / 2,
        y: (point1.y + point3.y) / 2,
        z: 0.04,
      };
      textPosition = coordinates.applyOffset(textPosition);
      const textRotationZ = Math.atan2(point2.y - point1.y, point2.x - point1.x);

      text.position.set(textPosition.x, textPosition.y, textPosition.z);
      text.rotation.set(0, 0, textRotationZ);
      text.visible = true;
      scene.add(text);
    }
    return text;
  }

  addZone(zone, color, coordinates, scene) {
    const drewObjects = [];

    const border = coordinates.applyOffsetToArray(zone.polygon.point);
    border.push(border[0]);

    const zoneMaterial = new THREE.MeshBasicMaterial({
      color,
      transparent: true,
      opacity: 0.15,
    });

    const zoneShape = drawShapeFromPoints(
      border, zoneMaterial, false, this.zOffsetFactor * 3, false,
    );
    scene.add(zoneShape);
    drewObjects.push(zoneShape);

    const mesh = drawSegmentsFromPoints(
      border, color, 2, this.zOffsetFactor, true, false, 1.0,
    );
    scene.add(mesh);
    drewObjects.push(mesh);

    return drewObjects;
  }

  addCurve(lines, color, coordinates, scene) {
    const drewObjects = [];
    lines.forEach((line) => {
      line.segment.forEach((segment) => {
        const points = coordinates.applyOffsetToArray(segment.lineSegment.point);
        const mesh = drawSegmentsFromPoints(
          points, color, 5, this.zOffsetFactor * 2, false,
        );
        scene.add(mesh);
        drewObjects.push(mesh);
      });
    });
    return drewObjects;
  }

  addStopLine(stopLine, coordinates, scene) {
    const drewObjects = this.addCurve(
      stopLine, colorMapping.PURE_WHITE, coordinates, scene,
    );
    return drewObjects;
  }

  removeDrewText(textMesh, scene) {
    if (textMesh) {
      textMesh.children.forEach((c) => c.visible = false);
      scene.remove(textMesh);
    }
  }

  removeDrewObjects(drewObjects, scene) {
    if (drewObjects) {
      drewObjects.forEach((object) => {
        scene.remove(object);
        if (object.geometry) {
          object.geometry.dispose();
        }
        if (object.material) {
          object.material.dispose();
        }
      });
    }
  }

  removeAllElements(scene) {
    this.removeExpiredElements([], scene);
    this.trafficSignals.removeAll(scene);
    this.stopSigns.removeAll(scene);
    this.yieldSigns.removeAll(scene);
  }

  removeExpiredElements(elementIds, scene) {
    const newData = {};
    for (const kind in this.data) {
      const drawThisKind = this.shouldDrawObjectOfThisElementKind(kind);
      newData[kind] = [];
      const oldDataOfThisKind = this.data[kind];
      const currentIds = elementIds[kind];
      oldDataOfThisKind.forEach((oldData) => {
        if (drawThisKind && currentIds && currentIds.includes(oldData.id.id)) {
          newData[kind].push(oldData);
        } else {
          this.removeDrewObjects(oldData.drewObjects, scene);
          this.removeDrewText(oldData.text, scene);
        }
      });
    }
    this.data = newData;
  }

  // I do not want to do premature optimization either. Should the
  // performance become an issue, all the diff should be done at the server
  // side. This also means that the server should maintain a state of
  // (possibly) visible elements, presummably in the global store.
  appendMapData(newData, coordinates, scene) {
    for (const kind in newData) {
      if (!newData[kind]) {
        continue;
      }

      if (!this.data[kind]) {
        this.data[kind] = [];
      }

      for (let i = 0; i < newData[kind].length; ++i) {
        switch (kind) {
          case 'lane':
            const lane = newData[kind][i];
            this.data[kind].push(Object.assign(newData[kind][i], {
              drewObjects: this.addLane(lane, coordinates, scene),
              text: this.addLaneId(lane, coordinates, scene),
            }));
            break;
          case 'clearArea':
            this.data[kind].push(Object.assign(newData[kind][i], {
              drewObjects: this.addZone(
                newData[kind][i], colorMapping.YELLOW, coordinates, scene,
              ),
            }));
            break;
          case 'crosswalk':
            this.data[kind].push(Object.assign(newData[kind][i], {
              drewObjects: this.addZone(
                newData[kind][i], colorMapping.PURE_WHITE, coordinates, scene,
              ),
            }));
            break;
          case 'junction':
            this.data[kind].push(Object.assign(newData[kind][i], {
              drewObjects: this.addBorder(
                newData[kind][i], colorMapping.BLUE, coordinates, scene,
              ),
            }));
            break;
          case 'pncJunction':
            this.data[kind].push(Object.assign(newData[kind][i], {
              drewObjects: this.addZone(
                newData[kind][i], colorMapping.BLUE, coordinates, scene,
              ),
            }));
            break;
          case 'signal':
            this.data[kind].push(Object.assign(newData[kind][i], {
              drewObjects: this.addStopLine(
                newData[kind][i].stopLine, coordinates, scene,
              ),
            }));
            this.trafficSignals.add([newData[kind][i]], coordinates, scene);
            break;
          case 'stopSign':
            this.data[kind].push(Object.assign(newData[kind][i], {
              drewObjects: this.addStopLine(
                newData[kind][i].stopLine, coordinates, scene,
              ),
            }));
            this.stopSigns.add([newData[kind][i]], coordinates, scene);
            break;
          case 'yield':
            this.data[kind].push(Object.assign(newData[kind][i], {
              drewObjects: this.addStopLine(
                newData[kind][i].stopLine, coordinates, scene,
              ),
            }));
            this.yieldSigns.add([newData[kind][i]], coordinates, scene);
            break;
          case 'road':
            const road = newData[kind][i];
            this.data[kind].push(Object.assign(newData[kind][i], {
              drewObjects: this.addRoad(road, coordinates, scene),
            }));
            break;
          case 'parkingSpace':
            this.data[kind].push(Object.assign(newData[kind][i], {
              drewObjects: this.addBorder(
                newData[kind][i], colorMapping.YELLOW, coordinates, scene,
              ),
              text: this.addParkingSpaceId(newData[kind][i], coordinates, scene),
            }));
            break;
          case 'speedBump':
            this.data[kind].push(Object.assign(newData[kind][i], {
              drewObjects: this.addCurve(
                newData[kind][i].position, colorMapping.RED, coordinates, scene,
              ),
            }));
            break;
          default:
            this.data[kind].push(newData[kind][i]);
            break;
        }
      }
    }
    return;
  }

  shouldDrawObjectOfThisElementKind(kind) {
    // Ex: mapping 'lane' to 'showMapLane' option
    const optionName = `showMap${kind[0].toUpperCase()}${kind.slice(1)}`;

    // NOTE: return true if the option is not found
    return STORE.options[optionName] !== false;
  }

  shouldDrawTextOfThisElementKind(kind) {
    // showMapLaneId option controls both laneId and parkingSpaceId
    return STORE.options.showMapLaneId && ['parkingSpace', 'lane'].includes(kind);
  }

  updateText() {
    for (const kind in this.data) {
      const isVisible = this.shouldDrawTextOfThisElementKind(kind);
      this.data[kind].forEach((element) => {
        if (element.text) {
          element.text.visible = isVisible;
        }
      });
    }
  }

  updateIndex(hash, elementIds, scene) {
    if (STORE.hmi.inNavigationMode) {
      MAP_WS.requestRelativeMapData();
    } else {
      this.updateText();

      let newElementKindsDrawn = '';
      for (const kind of Object.keys(elementIds).sort()) {
        if (this.shouldDrawObjectOfThisElementKind(kind)) {
          newElementKindsDrawn += kind;
        }
      }

      if (hash !== this.hash || this.elementKindsDrawn !== newElementKindsDrawn) {
        this.hash = hash;
        this.elementKindsDrawn = newElementKindsDrawn;
        const diff = this.diffMapElements(elementIds, this.data);
        if (!_.isEmpty(diff) || !this.initialized) {
          MAP_WS.requestMapData(diff);
          this.initialized = true;
        }

        this.removeExpiredElements(elementIds, scene);

        if (!this.shouldDrawObjectOfThisElementKind('signal')) {
          this.trafficSignals.removeAll(scene);
        } else {
          this.trafficSignals.removeExpired(elementIds.signal, scene);
        }

        if (!this.shouldDrawObjectOfThisElementKind('stopSign')) {
          this.stopSigns.removeAll(scene);
        } else {
          this.stopSigns.removeExpired(elementIds.stopSign, scene);
        }

        if (!this.shouldDrawObjectOfThisElementKind('yield')) {
          this.yieldSigns.removeAll(scene);
        } else {
          this.yieldSigns.removeExpired(elementIds.yield, scene);
        }
      }
    }
    // Do not set zOffset in camera view, since zOffset will affect the accuracy of matching
    // between hdmap and camera image
    this.zOffsetFactor = STORE.options.showCameraView ? 0 : 1;
  }

  update(world) {
    this.trafficSignals.updateTrafficLightStatus(world.perceivedSignal);
  }

  changeSelectedParkingSpaceColor(index, color = 0xff0000) {
    this.data.parkingSpace[index].drewObjects.forEach(mesh => {
      changeMaterial(mesh, color);
    });
  }
}
