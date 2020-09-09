import * as THREE from 'three';
import _ from 'lodash';

import STORE from 'store';
import Text3D from 'renderer/text3d';
import { copyProperty, hideArrayObjects, calculateLaneMarkerPoints } from 'utils/misc';
import {
  drawSegmentsFromPoints, drawDashedLineFromPoints,
  drawBox, drawSolidBox, drawDashedBox, drawArrow, drawImage, drawSolidPolygonFace,
} from 'utils/draw';

import iconObjectYield from 'assets/images/decision/object-yield.png';

const DEFAULT_HEIGHT = 1.5;
export const DEFAULT_COLOR = 0xFF00FC;
export const ObstacleColorMapping = {
  PEDESTRIAN: 0xFFEA00,
  BICYCLE: 0x00DCEB,
  VEHICLE: 0x00FF3C,
  VIRTUAL: 0x800000,
  CIPV: 0xFF9966,
};
const LINE_THICKNESS = 1.5;
const FACE_TYPE = Object.freeze({
  SOLID_LINE: 'extrusionSolidFaces',
  DASHED_LINE: 'extrusionDashedFaces',
  SOLID_FACE: 'v2xSolidFaces',
});
const CUBE_TYPE = Object.freeze({
  SOLID_LINE: 'solidCubes',
  DASHED_LINE: 'dashedCubes',
  SOLID_FACE: 'v2xCubes',
});

export default class PerceptionObstacles {
  constructor() {
    this.textRender = new Text3D();
    this.arrows = []; // for indication of direction of moving obstacles
    this.ids = []; // for obstacle id labels
    this.solidCubes = []; // for obstacles with only length/width/height
    this.dashedCubes = []; // for obstacles with only length/width/height
    this.extrusionSolidFaces = []; // for obstacles with polygon points
    this.extrusionDashedFaces = []; // for obstacles with polygon points
    this.laneMarkers = []; // for lane markers
    this.icons = [];
    this.trafficCones = []; // for traffic cone meshes
    this.v2xCubes = [];
    this.v2xSolidFaces = [];

    this.arrowIdx = 0;
    this.cubeIdx = 0;
    this.extrusionFaceIdx = 0;
    this.iconIdx = 0;
    this.trafficConeIdx = 0;
    this.v2xCubeIdx = 0;
    this.v2xSolidFaceIdx = 0;
  }

  update(world, coordinates, scene, isBirdView) {
    this.resetObjects(scene, _.isEmpty(world.object));
    this.updateObjects(world, coordinates, scene, isBirdView);
    this.updateSensorMeasurements(world, coordinates, scene);
    this.hideUnusedObjects();

    this.updateLaneMarkers(world, coordinates, scene);
  }

  updateObjects(world, coordinates, scene, isBirdView) {
    const objects = world.object;
    if (_.isEmpty(objects)) {
      return;
    }

    const adc = coordinates.applyOffset({
      x: world.autoDrivingCar.positionX,
      y: world.autoDrivingCar.positionY,
    });
    adc.heading = world.autoDrivingCar.heading;

    for (let i = 0; i < objects.length; i++) {
      const obstacle = objects[i];
      if (!STORE.options[`showObstacles${_.upperFirst(_.camelCase(obstacle.type))}`]
        || !_.isNumber(obstacle.positionX) || !_.isNumber(obstacle.positionY)) {
        continue;
      }

      if (!STORE.options.showObstaclesV2xInfo && obstacle.source === 'V2X') {
        continue;
      }

      const position = coordinates.applyOffset(
        new THREE.Vector3(obstacle.positionX,
          obstacle.positionY,
          (obstacle.height || DEFAULT_HEIGHT) / 2),
      );
      const color = ObstacleColorMapping[obstacle.type] || DEFAULT_COLOR;
      const isV2X = (obstacle.source === 'V2X');

      if (STORE.options.showObstaclesVelocity && obstacle.type
        && obstacle.type !== 'UNKNOWN_UNMOVABLE' && obstacle.speed > 0.5) {
        const arrowMesh = this.updateArrow(position,
          obstacle.speedHeading, color, scene);
        this.arrowIdx++;
        const scale = 1 + Math.log2(obstacle.speed);
        arrowMesh.scale.set(scale, scale, scale);
        arrowMesh.visible = true;
      }

      if (STORE.options.showObstaclesHeading) {
        this.drawObstacleHeading(position, obstacle.heading, scene);
        this.arrowIdx++;
      }

      this.updateTexts(adc, obstacle, position, scene, isBirdView, isV2X);

      // get the confidence and validate its range
      let confidence = obstacle.confidence;
      confidence = Math.max(0.0, confidence);
      confidence = Math.min(1.0, confidence);
      const polygon = obstacle.polygonPoint;

      if (obstacle.subType === 'ST_TRAFFICCONE') {
        this.updateTrafficCone(position, scene);
        this.trafficConeIdx++;
      } else if (polygon !== undefined && polygon.length > 0) {
        if (isV2X) {
          this.updatePolygon(polygon, obstacle.height, color, coordinates, confidence,
            scene, true);
          this.v2xSolidFaceIdx += polygon.length;
        } else {
          this.updatePolygon(polygon, obstacle.height, color, coordinates, confidence,
            scene, false);
          this.extrusionFaceIdx += polygon.length;
        }
      } else if (obstacle.length && obstacle.width && obstacle.height) {
        if (isV2X) {
          this.updateV2xCube(obstacle.length, obstacle.width, obstacle.height, position,
            obstacle.heading, color, scene);
          this.v2xCubeIdx++;
        } else {
          this.updateCube(obstacle.length, obstacle.width, obstacle.height, position,
            obstacle.heading, color, confidence, scene);
          this.cubeIdx++;
        }
      }


      // draw a yield sign to indicate ADC is yielding to this obstacle
      if (obstacle.yieldedObstacle) {
        const iconPosition = {
          x: position.x,
          y: position.y,
          z: position.z + obstacle.height + 0.5,
        };
        this.updateIcon(iconPosition, world.autoDrivingCar.heading, scene);
        this.iconIdx++;
      }
    }
  }

  updateSensorMeasurements(world, coordinates, scene) {
    if (!STORE.options.showObstaclesLidarSensor && !STORE.options.showObstaclesRadarSensor
      && !STORE.options.showObstaclesCameraSensor) {
      return;
    }

    const sensorMeasures = world.sensorMeasurements;
    for (const key in sensorMeasures) {
      const sensorType = this.deduceSensorType(key.toLowerCase());
      if (!sensorType || !STORE.options[`showObstacles${sensorType}`]) {
        continue;
      }

      for (const measurement of sensorMeasures[key].sensorMeasurement) {
        if (!_.isNumber(measurement.positionX) || !_.isNumber(measurement.positionY)) {
          continue;
        }

        const position = coordinates.applyOffset(
          new THREE.Vector3(measurement.positionX,
            measurement.positionY,
            (measurement.height || DEFAULT_HEIGHT) / 2),
        );
        const color = ObstacleColorMapping[measurement.type] || DEFAULT_COLOR;

        if (STORE.options.showObstaclesHeading) {
          this.drawObstacleHeading(position, measurement.heading, scene);
          this.arrowIdx++;
        }

        if (measurement.subType === 'ST_TRAFFICCONE') {
          this.updateTrafficCone(position, scene);
          this.trafficConeIdx++;
        } else if (measurement.length && measurement.width && measurement.height) {
          this.updateCube(measurement.length, measurement.width,
            measurement.height, position,
            measurement.heading, color, 0.5, scene);
          this.cubeIdx++;
        }
      }
    }
  }

  resetObjects(scene, empty) {
    // Id meshes need to be recreated every time.
    // Each text mesh needs to be removed from the scene,
    // and its char meshes need to be hidden for reuse purpose.
    if (!_.isEmpty(this.ids)) {
      this.ids.forEach((t) => {
        t.children.forEach((c) => c.visible = false);
        scene.remove(t);
      });
      this.ids = [];
    }

    this.textRender.reset();
    this.arrowIdx = 0;
    this.cubeIdx = 0;
    this.extrusionFaceIdx = 0;
    this.iconIdx = 0;
    this.trafficConeIdx = 0;
    this.v2xCubeIdx = 0;
    this.v2xSolidFaceIdx = 0;
    if (empty) {
      this.hideUnusedObjects();
    }
  }

  hideUnusedObjects() {
    hideArrayObjects(this.arrows, this.arrowIdx);
    hideArrayObjects(this.solidCubes, this.cubeIdx);
    hideArrayObjects(this.dashedCubes, this.cubeIdx);
    hideArrayObjects(this.extrusionSolidFaces, this.extrusionFaceIdx);
    hideArrayObjects(this.extrusionDashedFaces, this.extrusionFaceIdx);
    hideArrayObjects(this.icons, this.iconIdx);
    hideArrayObjects(this.trafficCones, this.trafficConeIdx);
    hideArrayObjects(this.v2xCubes, this.v2xCubeIdx);
    hideArrayObjects(this.v2xSolidFaces, this.v2xSolidFaceIdx);
  }

  deduceSensorType(key) {
    if (key.search('radar') !== -1) {
      return 'RadarSensor';
    }
    if (key.search('lidar') !== -1 || key.search('velodyne') !== -1) {
      return 'LidarSensor';
    }
    if (key.search('camera') !== -1) {
      return 'CameraSensor';
    }
    console.warn('Cannot deduce sensor type:', key);
    return null;
  }

  updateArrow(position, heading, color, scene) {
    const arrowMesh = this.getArrow(this.arrowIdx, scene);
    copyProperty(arrowMesh.position, position);
    arrowMesh.material.color.setHex(color);
    arrowMesh.rotation.set(0, 0, -(Math.PI / 2 - heading));
    return arrowMesh;
  }

  updateTexts(adc, obstacle, obstaclePosition, scene, isBirdView, isV2X) {
    const initPosition = {
      x: obstaclePosition.x,
      y: obstaclePosition.y,
      z: obstacle.height || 3,
    };

    const lineSpacing = 0.5;
    const deltaX = isBirdView ? 0.0 : lineSpacing * Math.cos(adc.heading);
    const deltaY = isBirdView ? 0.7 : lineSpacing * Math.sin(adc.heading);
    const deltaZ = isBirdView ? 0.0 : lineSpacing;
    let lineCount = 0;
    if (STORE.options.showObstaclesInfo) {
      const distance = adc.distanceTo(obstaclePosition).toFixed(1);
      const speed = obstacle.speed.toFixed(1);
      this.drawTexts(`(${distance}m, ${speed}m/s)`, initPosition, scene);
      lineCount++;
    }
    if (STORE.options.showObstaclesId) {
      const textPosition = {
        x: initPosition.x + (lineCount * deltaX),
        y: initPosition.y + (lineCount * deltaY),
        z: initPosition.z + (lineCount * deltaZ),
      };
      this.drawTexts(obstacle.id, textPosition, scene);
      lineCount++;
    }
    if (STORE.options.showPredictionPriority) {
      const priority = _.get(obstacle, 'obstaclePriority.priority');
      if (priority && priority !== 'NORMAL') {
        const textPosition = {
          x: initPosition.x + (lineCount * deltaX),
          y: initPosition.y + (lineCount * deltaY),
          z: initPosition.z + (lineCount * deltaZ),
        };
        this.drawTexts(priority, textPosition, scene);
        lineCount++;
      }
    }
    if (isV2X) {
      _.get(obstacle,'v2xInfo.v2xType',[]).forEach((t) => {
        const textPosition = {
          x: initPosition.x + (lineCount * deltaX),
          y: initPosition.y + (lineCount * deltaY),
          z: initPosition.z + (lineCount * deltaZ),
        };
        this.drawTexts(t, textPosition, scene, 0xFF0000);
        lineCount++;
      });
    }
  }

  updatePolygon(points, height, color, coordinates, confidence, scene, isForV2X = false) {
    for (let i = 0; i < points.length; i++) {
      // Get the adjacent point.
      const next = (i === points.length - 1) ? 0 : i + 1;
      const v = new THREE.Vector3(points[i].x, points[i].y, points[i].z);
      const vNext = new THREE.Vector3(points[next].x, points[next].y, points[next].z);

      // Compute position.
      const facePosition = coordinates.applyOffset(
        new THREE.Vector2((v.x + vNext.x) / 2.0, (v.y + vNext.y) / 2.0),
      );
      if (facePosition === null) {
        continue;
      }

      // Compute face scale.
      const edgeDistance = v.distanceTo(vNext);
      if (edgeDistance === 0) {
        console.warn('Cannot display obstacle with an edge length 0!');
        continue;
      }

      if (isForV2X) {
        const v2xFaceMesh = this.getFace(this.v2xSolidFaceIdx + i, scene, FACE_TYPE.SOLID_FACE);
        v2xFaceMesh.position.set(facePosition.x, facePosition.y, height);
        v2xFaceMesh.scale.set(edgeDistance, 1, height);
        v2xFaceMesh.material.color.setHex(color);
        v2xFaceMesh.rotation.set(0, 0, Math.atan2(vNext.y - v.y, vNext.x - v.x));
        // Make the plane stand up
        v2xFaceMesh.rotateX(Math.PI / 2);
        v2xFaceMesh.visible = true;
      } else {
        const solidFaceMesh = this.getFace(this.extrusionFaceIdx + i, scene, FACE_TYPE.SOLID_LINE);
        const dashedFaceMesh = this.getFace(this.extrusionFaceIdx + i, scene,
          FACE_TYPE.DASHED_LINE);
        solidFaceMesh.position.set(facePosition.x, facePosition.y, 0);
        dashedFaceMesh.position.set(facePosition.x, facePosition.y, height * confidence);
        solidFaceMesh.scale.set(edgeDistance, 1, height * confidence);
        dashedFaceMesh.scale.set(edgeDistance, 1, height * (1 - confidence));
        solidFaceMesh.material.color.setHex(color);
        solidFaceMesh.rotation.set(0, 0, Math.atan2(vNext.y - v.y, vNext.x - v.x));
        solidFaceMesh.visible = (confidence !== 0.0);
        dashedFaceMesh.material.color.setHex(color);
        dashedFaceMesh.rotation.set(0, 0, Math.atan2(vNext.y - v.y, vNext.x - v.x));
        dashedFaceMesh.visible = (confidence !== 1.0);
      }

    }
  }

  updateV2xCube(length, width, height, position, heading, color, scene) {
    const v2xCubeMesh = this.getCube(this.v2xCubeIdx, scene, CUBE_TYPE.SOLID_FACE);
    v2xCubeMesh.position.set(
      position.x, position.y, position.z);
    v2xCubeMesh.scale.set(length, width, height);
    v2xCubeMesh.material.color.setHex(color);
    // Change the outline color
    v2xCubeMesh.children[0].material.color.setHex(color);
    v2xCubeMesh.rotation.set(0, 0, heading);
    v2xCubeMesh.visible = true;
  }

  updateCube(length, width, height, position, heading, color, confidence, scene) {
    if (confidence > 0) {
      const solidCubeMesh = this.getCube(this.cubeIdx, scene, CUBE_TYPE.SOLID_LINE);
      solidCubeMesh.position.set(
        position.x, position.y, position.z + height * (confidence - 1) / 2);
      solidCubeMesh.scale.set(length, width, height * confidence);
      solidCubeMesh.material.color.setHex(color);
      solidCubeMesh.rotation.set(0, 0, heading);
      solidCubeMesh.visible = true;
    }

    if (confidence < 1) {
      const dashedCubeMesh = this.getCube(this.cubeIdx, scene, CUBE_TYPE.DASHED_LINE);
      dashedCubeMesh.position.set(
        position.x, position.y, position.z + height * confidence / 2);
      dashedCubeMesh.scale.set(length, width, height * (1 - confidence));
      dashedCubeMesh.material.color.setHex(color);
      dashedCubeMesh.rotation.set(0, 0, heading);
      dashedCubeMesh.visible = true;
    }
  }

  updateIcon(position, heading, scene) {
    const icon = this.getIcon(this.iconIdx, scene);
    copyProperty(icon.position, position);
    icon.rotation.set(Math.PI / 2, heading - Math.PI / 2, 0);
    icon.visible = true;
  }

  updateTrafficCone(position, scene) {
    const cone = this.getTrafficCone(this.trafficConeIdx, scene);
    cone.position.setX(position.x);
    cone.position.setY(position.y);
    cone.visible = true;
  }

  getArrow(index, scene) {
    if (index < this.arrows.length) {
      return this.arrows[index];
    }
    const arrowMesh = drawArrow(1.5, LINE_THICKNESS, 0.5, 0.5, DEFAULT_COLOR);
    arrowMesh.rotation.set(0, 0, -Math.PI / 2);
    arrowMesh.visible = false;
    this.arrows.push(arrowMesh);
    scene.add(arrowMesh);
    return arrowMesh;
  }

  getFace(index, scene, type) {
    const extrusionFaces = this[type];
    if (index < extrusionFaces.length) {
      return extrusionFaces[index];
    }

    const points = [
      new THREE.Vector3(-0.5, 0, 0),
      new THREE.Vector3(0.5, 0, 0),
      new THREE.Vector3(0.5, 0, 1),
      new THREE.Vector3(-0.5, 0, 1),
    ];
    let extrusionFace = null;
    switch (type) {
      case FACE_TYPE.SOLID_FACE:
        extrusionFace = drawSolidPolygonFace();
        break;
      case FACE_TYPE.SOLID_LINE:
        extrusionFace = drawSegmentsFromPoints(points, DEFAULT_COLOR, LINE_THICKNESS);
        break;
      default:
        extrusionFace = drawDashedLineFromPoints(points, DEFAULT_COLOR, LINE_THICKNESS, 0.1, 0.1);
        break;
    }
    extrusionFace.visible = false;
    extrusionFaces.push(extrusionFace);
    scene.add(extrusionFace);
    return extrusionFace;
  }

  getCube(index, scene, type) {
    const cubes = this[type];
    if (index < cubes.length) {
      return cubes[index];
    }
    const cubeSize = new THREE.Vector3(1, 1, 1);
    let cubeMesh = null;
    switch (type) {
      case CUBE_TYPE.SOLID_FACE:
        cubeMesh = drawSolidBox(cubeSize, DEFAULT_COLOR, LINE_THICKNESS);
        break;
      case CUBE_TYPE.SOLID_LINE:
        cubeMesh = drawBox(cubeSize, DEFAULT_COLOR, LINE_THICKNESS);
        break;
      default:
        cubeMesh = drawDashedBox(cubeSize, DEFAULT_COLOR, LINE_THICKNESS, 0.1, 0.1);
        break;
    }
    cubeMesh.visible = false;
    cubes.push(cubeMesh);
    scene.add(cubeMesh);
    return cubeMesh;
  }

  getIcon(index, scene) {
    if (index < this.icons.length) {
      return this.icons[index];
    }
    const icon = drawImage(iconObjectYield, 1, 1, 3, 3.6, 0);
    icon.rotation.set(0, 0, -Math.PI / 2);
    icon.visible = false;
    this.icons.push(icon);
    scene.add(icon);
    return icon;
  }

  drawTexts(content, position, scene, color = 0xFFEA00) {
    const text = this.textRender.drawText(content, scene, color);
    if (text) {
      text.position.set(position.x, position.y, position.z);
      this.ids.push(text);
      scene.add(text);
    }
  }

  drawObstacleHeading(position, heading, scene) {
    const arrowMesh = this.updateArrow(position, heading, 0xFFFFFF, scene);
    arrowMesh.scale.set(1, 1, 1);
    arrowMesh.visible = true;
  }

  updateLaneMarkers(world, coordinates, scene) {
    if (!_.isEmpty(this.laneMarkers)) {
      this.laneMarkers.forEach((laneMesh) => {
        scene.remove(laneMesh);
        laneMesh.geometry.dispose();
        laneMesh.material.dispose();
      });
      this.laneMarkers = [];
    }

    if (STORE.options.showPerceptionLaneMarker) {
      const adc = world.autoDrivingCar;
      for (const name in world.laneMarker) {
        const absolutePoints = calculateLaneMarkerPoints(adc, world.laneMarker[name]);
        if (absolutePoints.length) {
          const offsetPoints = absolutePoints.map(
            (point) => coordinates.applyOffset(point));
          const mesh = drawSegmentsFromPoints(offsetPoints, 0x006AFF, 2, 4, false);
          scene.add(mesh);
          this.laneMarkers.push(mesh);
        }
      }
    }
  }

  getTrafficCone(index, scene) {
    if (index < this.trafficCones.length) {
      return this.trafficCones[index];
    }

    const height = 0.914;
    const geometry = new THREE.CylinderGeometry(0.1, 0.25, height, 32);
    const material = new THREE.MeshBasicMaterial({
      color: 0xE1601C,
      transparent: true,
      opacity: 0.65,
    });
    const cone = new THREE.Mesh(geometry, material);
    cone.rotation.set(Math.PI / 2, 0, 0);
    cone.position.set(0, 0, height / 2);
    this.trafficCones.push(cone);
    scene.add(cone);

    return cone;
  }
}
