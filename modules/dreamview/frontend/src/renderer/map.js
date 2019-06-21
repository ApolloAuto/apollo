import * as THREE from "three";
import STORE from "store";
import { MAP_WS } from "store/websocket";
import _ from "lodash";

import {
    drawSegmentsFromPoints,
    drawDashedLineFromPoints,
    drawShapeFromPoints
} from "utils/draw";
import Text3D, { TEXT_ALIGN } from "renderer/text3d";

import stopSignMaterial from "assets/models/stop_sign.mtl";
import stopSignObject from "assets/models/stop_sign.obj";

import trafficLightMaterial from "assets/models/traffic_light.mtl";
import trafficLightObject from "assets/models/traffic_light.obj";

import { loadObject } from "utils/models";

const colorMapping = {
    YELLOW: 0XDAA520,
    WHITE: 0xCCCCCC,
    CORAL: 0xFF7F50,
    RED: 0xFF6666,
    GREEN: 0x006400,
    BLUE: 0x30A5FF,
    PURE_WHITE: 0xFFFFFF,
    DEFAULT: 0xC0C0C0
};

const TRAFFIC_LIGHT_SCALE = 0.006;
export const trafficLightScales = {
    x: TRAFFIC_LIGHT_SCALE,
    y: TRAFFIC_LIGHT_SCALE,
    z: TRAFFIC_LIGHT_SCALE
};

const STOP_SIGN_SCALE = 0.01;
export const stopSignScales = {
    x: STOP_SIGN_SCALE,
    y: STOP_SIGN_SCALE,
    z: STOP_SIGN_SCALE
};

const EPSILON = 1e-9;
const Z_OFFSET_FACTOR = 1;

export default class Map {
    constructor() {
        this.textRender = new Text3D();
        this.hash = -1;
        this.data = {};
        this.initialized = false;
        this.elementKindsDrawn = '';
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
                const found = oldData ? oldData.find(old => {
                    return old.id.id === newIds[i];
                }) : false;

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
            case "DOTTED_YELLOW":
                return drawDashedLineFromPoints(
                    points, colorMapping.YELLOW, 4, 3, 3, Z_OFFSET_FACTOR, 1, false);
            case "DOTTED_WHITE":
                return drawDashedLineFromPoints(
                    points, colorMapping.WHITE, 2, 0.5, 0.25, Z_OFFSET_FACTOR, 0.4, false);
            case "SOLID_YELLOW":
                return drawSegmentsFromPoints(
                    points, colorMapping.YELLOW, 3, Z_OFFSET_FACTOR, false);
            case "SOLID_WHITE":
                return drawSegmentsFromPoints(
                    points, colorMapping.WHITE, 3, Z_OFFSET_FACTOR, false);
            case "DOUBLE_YELLOW":
                const left = drawSegmentsFromPoints(
                    points, colorMapping.YELLOW, 2, Z_OFFSET_FACTOR, false);
                const right = drawSegmentsFromPoints(
                    points.map(point =>
                        new THREE.Vector3(point.x + 0.3, point.y + 0.3, point.z)),
                    colorMapping.YELLOW, 3, Z_OFFSET_FACTOR, false);
                left.add(right);
                return left;
            case "CURB":
                return drawSegmentsFromPoints(
                    points, colorMapping.CORAL, 3, Z_OFFSET_FACTOR, false);
            default:
                return drawSegmentsFromPoints(
                    points, colorMapping.DEFAULT, 3, Z_OFFSET_FACTOR, false);
        }
    }

    addLane(lane, coordinates, scene) {
        const drewObjects = [];

        const centralLine = lane.centralCurve.segment;
        centralLine.forEach(segment => {
            const points = coordinates.applyOffsetToArray(segment.lineSegment.point);
            const centerLine =
                drawSegmentsFromPoints(points, colorMapping.GREEN, 1, Z_OFFSET_FACTOR, false);
            centerLine.name = "CentralLine-" + lane.id.id;
            scene.add(centerLine);
            drewObjects.push(centerLine);
        });

        const rightLaneType = lane.rightBoundary.boundaryType[0].types[0];
        // TODO: this is a temp. fix for repeated boundary types.
        lane.rightBoundary.curve.segment.forEach((segment, index) => {
            const points = coordinates.applyOffsetToArray(segment.lineSegment.point);
            const boundary = this.addLaneMesh(rightLaneType, points);
            boundary.name = "RightBoundary-" + lane.id.id;
            scene.add(boundary);
            drewObjects.push(boundary);
        });

        const leftLaneType = lane.leftBoundary.boundaryType[0].types[0];
        lane.leftBoundary.curve.segment.forEach((segment, index) => {
            const points = coordinates.applyOffsetToArray(segment.lineSegment.point);
            const boundary = this.addLaneMesh(leftLaneType, points);
            boundary.name = "LeftBoundary-" + lane.id.id;
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
            lane.id.id, scene, colorMapping.WHITE, TEXT_ALIGN.LEFT);
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

        road.section.forEach(section => {
            section.boundary.outerPolygon.edge.forEach(edge => {
                edge.curve.segment.forEach((segment, index) => {
                    const points = coordinates.applyOffsetToArray(segment.lineSegment.point);
                    const boundary = this.addLaneMesh("CURB", points);
                    boundary.name = "Road-" + road.id.id;
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
            border, color, 2, Z_OFFSET_FACTOR, true, false, 1.0);
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
                z: 0.04
            };
            textPosition = coordinates.applyOffset(textPosition);
            const textRotationZ = Math.atan2(point2.y - point1.y, point2.x - point1.x);

            text.position.set(textPosition.x, textPosition.y, textPosition.z);
            text.rotation.set(0, 0, textRotationZ);
            text.visible = false;
            scene.add(text);
        }
        return text;
    }

    addZone(zone, color, coordinates, scene) {
        const drewObjects = [];

        const border = coordinates.applyOffsetToArray(zone.polygon.point);
        border.push(border[0]);

        const zoneMaterial = new THREE.MeshBasicMaterial({
            color: color,
            transparent: true,
            opacity: .15
        });

        const zoneShape = drawShapeFromPoints(border, zoneMaterial, false, 3, false);
        scene.add(zoneShape);
        drewObjects.push(zoneShape);

        const mesh = drawSegmentsFromPoints(
            border, color, 2, Z_OFFSET_FACTOR, true, false, 1.0);
        scene.add(mesh);
        drewObjects.push(mesh);

        return drewObjects;
    }

    addCurve(lines, color, coordinates, scene) {
        const drewObjects = [];
        lines.forEach(line => {
            line.segment.forEach(segment => {
                const points = coordinates.applyOffsetToArray(segment.lineSegment.point);
                const mesh = drawSegmentsFromPoints(points, color, 5, Z_OFFSET_FACTOR + 1, false);
                scene.add(mesh);
                drewObjects.push(mesh);
            });
        });
        return drewObjects;
    }

    getHeadingFromStopLine(object) {
        const stopLine = object.stopLine[0].segment[0].lineSegment.point;
        const len = stopLine.length;
        if (len >= 2) {
            const stopLineDirection = Math.atan2(stopLine[len - 1].y - stopLine[0].y,
                stopLine[len - 1].x - stopLine[0].x);
            return Math.PI * 1.5 + stopLineDirection;
        }
        return NaN;
    }

    // use the signal real direction to decide its heading
    // but the signal could either face towards or away from the stop line
    // so use the intersection of signal's orthiogonal line and stop line to decide
    getHeadingFromStopLineAndTrafficLightBoundary(signal) {
        // find the plane of the signal
        const boundaryPoints = signal.boundary.point;
        if (boundaryPoints.length < 3) {
            console.warn("Cannot get three points from boundary, signal_id: " + signal.id.id);
            return this.getHeadingFromStopLine(signal);
        }
        const boundary1 = boundaryPoints[0];
        const boundary2 = boundaryPoints[1];
        const boundary3 = boundaryPoints[2];
        // get an orthogonal line of the plane (only need its projection on XY coordinate system)
        // construct ax+by+c=0 ==> orthogonalX*x+orthogonalY*y+constant=0
        const orthogonalX = (boundary2.x - boundary1.x) * (boundary3.z - boundary1.z)
            - (boundary3.x - boundary1.x) * (boundary2.z - boundary1.z);
        const orthogonalY = (boundary2.y - boundary1.y) * (boundary3.z - boundary1.z)
            - (boundary3.y - boundary1.y) * (boundary2.z - boundary1.z);
        const orthogonalConstant = -orthogonalX * boundary1.x - orthogonalY * boundary1.y;
        // get the stop line
        const stopLine = _.get(signal, 'stopLine[0].segment[0].lineSegment.point', '');
        const len = stopLine.length;
        if (len < 2) {
            console.warn("Cannot get any stop line, signal_id: " + signal.id.id);
            return NaN;
        }
        // construct ax+by+c=0 ==> stopLineX*x+stopLineY*y+constant=0
        const stopLineX = stopLine[len - 1].y - stopLine[0].y;
        const stopLineY = stopLine[0].x - stopLine[len - 1].x;
        const stopLineConstant = -stopLineX * stopLine[0].x - stopLineY * stopLine[0].y;
        // calculate the intersection
        if (Math.abs(stopLineX * orthogonalY - orthogonalX * stopLineY) < EPSILON) {
            console.warn("The signal orthogonal direction is parallel to the stop line,",
                "signal_id: " + signal.id.id);
            return this.getHeadingFromStopLine(signal);
        }
        const intersectX = (stopLineY * orthogonalConstant - orthogonalY * stopLineConstant)
            / (stopLineX * orthogonalY - orthogonalX * stopLineY);
        const intersectY = stopLineY !== 0 ?
            (-stopLineX * intersectX - stopLineConstant) / stopLineY
            : (-orthogonalX * intersectX - orthogonalConstant) / orthogonalY;
        let direction = Math.atan2(- orthogonalX, orthogonalY);
        // if the direction is not towards to intersection point, turn around
        if ((direction < 0 && intersectY > boundary1.y) ||
            (direction > 0 && intersectY < boundary1.y)) {
            direction += Math.PI;
        }
        return direction;
    }

    getSignalPositionAndHeading(signal, coordinates) {
        const locations = [];
        signal.subsignal.forEach(subsignal => {
            if (subsignal.location) {
                locations.push(subsignal.location);
            }
        });
        if (locations.length === 0) {
            console.warn("Subsignal locations not found, use signal boundary instead.");
            locations.push(signal.boundary.point);
        }
        if (locations.length === 0) {
            console.warn("Unable to determine signal location, skip.");
            return null;
        }
        const heading = this.getHeadingFromStopLineAndTrafficLightBoundary(signal);
        if (!isNaN(heading)) {
            let position = new THREE.Vector3(0, 0, 0);
            position.x = _.meanBy(_.values(locations), l => l.x);
            position.y = _.meanBy(_.values(locations), l => l.y);
            position = coordinates.applyOffset(position);
            return { "pos": position, "heading": heading };
        } else {
            console.error('Error loading traffic light. Unable to determine heading.');
            return null;
        }
    }

    addTrafficLight(signal, coordinates, scene) {
        // Draw stop line
        const drewObjects = this.addCurve(
            signal.stopLine, colorMapping.PURE_WHITE, coordinates, scene);

        // Add traffic light object
        const posAndHeading = this.getSignalPositionAndHeading(signal, coordinates);
        if (posAndHeading) {
            loadObject(trafficLightMaterial, trafficLightObject,
                trafficLightScales,
                mesh => {
                    mesh.rotation.x = Math.PI / 2;
                    mesh.rotation.y = posAndHeading.heading;
                    mesh.position.set(posAndHeading.pos.x, posAndHeading.pos.y, 0);
                    mesh.matrixAutoUpdate = false;
                    mesh.updateMatrix();

                    scene.add(mesh);
                    drewObjects.push(mesh);
                });
        }
        return drewObjects;
    }

    getStopSignPositionAndHeading(stopSign, coordinates) {
        const heading = this.getHeadingFromStopLine(stopSign);

        if (!isNaN(heading)) {
            const stopLinePoint = _.last(stopSign.stopLine[0].segment[0].lineSegment.point);
            let position = new THREE.Vector3(stopLinePoint.x, stopLinePoint.y, 0);
            position = coordinates.applyOffset(position);

            return { "pos": position, "heading": heading };
        } else {
            console.error('Error loading stop sign. Unable to determine heading.');
            return null;
        }
    }

    addStopSign(stopSign, coordinates, scene) {
        // Draw stop line
        const drewObjects = this.addCurve(
            stopSign.stopLine, colorMapping.PURE_WHITE, coordinates, scene);

        // Add stop sign object
        const posAndHeading = this.getStopSignPositionAndHeading(stopSign, coordinates);
        if (posAndHeading) {
            loadObject(stopSignMaterial, stopSignObject, stopSignScales,
                mesh => {
                    mesh.rotation.x = Math.PI / 2;
                    mesh.rotation.y = posAndHeading.heading - Math.PI / 2;
                    mesh.position.set(posAndHeading.pos.x, posAndHeading.pos.y, 0);
                    mesh.matrixAutoUpdate = false;
                    mesh.updateMatrix();
                    scene.add(mesh);
                    drewObjects.push(mesh);
                });
        }
        return drewObjects;
    }

    removeDrewText(textMesh, scene) {
        if (textMesh) {
            textMesh.children.forEach(c => c.visible = false);
            scene.remove(textMesh);
        }
    }

    removeDrewObjects(drewObjects, scene) {
        if (drewObjects) {
            drewObjects.forEach(object => {
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
    }

    removeExpiredElements(elementIds, scene) {
        const newData = {};
        for (const kind in this.data) {
            const drawThisKind = this.shouldDrawObjectOfThisElementKind(kind);
            newData[kind] = [];
            const oldDataOfThisKind = this.data[kind];
            const currentIds = elementIds[kind];
            oldDataOfThisKind.forEach(oldData => {
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
                    case "lane":
                        const lane = newData[kind][i];
                        this.data[kind].push(Object.assign(newData[kind][i], {
                            drewObjects: this.addLane(lane, coordinates, scene),
                            text: this.addLaneId(lane, coordinates, scene)
                        }));
                        break;
                    case "clearArea":
                        this.data[kind].push(Object.assign(newData[kind][i], {
                            drewObjects: this.addZone(
                                newData[kind][i], colorMapping.YELLOW, coordinates, scene)
                        }));
                        break;
                    case "crosswalk":
                        this.data[kind].push(Object.assign(newData[kind][i], {
                            drewObjects: this.addZone(
                                newData[kind][i], colorMapping.PURE_WHITE, coordinates, scene)
                        }));
                        break;
                    case "junction":
                        this.data[kind].push(Object.assign(newData[kind][i], {
                            drewObjects: this.addBorder(
                                newData[kind][i], colorMapping.BLUE, coordinates, scene)
                        }));
                        break;
                    case "pncJunction":
                        this.data[kind].push(Object.assign(newData[kind][i], {
                            drewObjects: this.addZone(
                                newData[kind][i], colorMapping.BLUE, coordinates, scene)
                        }));
                        break;
                    case "signal":
                        this.data[kind].push(Object.assign(newData[kind][i], {
                            drewObjects: this.addTrafficLight(
                                newData[kind][i], coordinates, scene)
                        }));
                        break;
                    case "stopSign":
                        this.data[kind].push(Object.assign(newData[kind][i], {
                            drewObjects: this.addStopSign(
                                newData[kind][i], coordinates, scene)
                        }));
                        break;
                    case "road":
                        const road = newData[kind][i];
                        this.data[kind].push(Object.assign(newData[kind][i], {
                            drewObjects: this.addRoad(road, coordinates, scene)
                        }));
                        break;
                    case "parkingSpace":
                        this.data[kind].push(Object.assign(newData[kind][i], {
                            drewObjects: this.addBorder(
                                newData[kind][i], colorMapping.YELLOW, coordinates, scene),
                            text: this.addParkingSpaceId(newData[kind][i], coordinates, scene)
                        }));
                        break;
                    case "speedBump":
                        this.data[kind].push(Object.assign(newData[kind][i], {
                            drewObjects: this.addCurve(
                                newData[kind][i].position, colorMapping.RED, coordinates, scene)
                        }));
                        break;
                    default:
                        this.data[kind].push(newData[kind][i]);
                        break;
                }
            }
        }
    }

    shouldDrawObjectOfThisElementKind(kind) {
        // Ex: mapping 'lane' to 'showMapLane' option
        const optionName = `showMap${kind[0].toUpperCase()}${kind.slice(1)}`;

        // NOTE: return true if the option is not found
        return STORE.options[optionName] !== false;
    }

    shouldDrawTextOfThisElementKind(kind) {
        // showMapLaneId option controls both laneId and parkingSpaceId
        return STORE.options['showMapLaneId'] && ['parkingSpace', 'lane'].includes(kind);
    }

    updateText() {
        for (const kind in this.data) {
            const isVisible = this.shouldDrawTextOfThisElementKind(kind);
            this.data[kind].forEach(element => {
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
                this.removeExpiredElements(elementIds, scene);
                if (!_.isEmpty(diff) || !this.initialized) {
                    MAP_WS.requestMapData(diff);
                    this.initialized = true;
                }
            }
        }
    }
}

