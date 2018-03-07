import * as THREE from "three";
import STORE from "store";
import { MAP_WS } from "store/websocket";

import {
    drawSegmentsFromPoints,
    drawDashedLineFromPoints,
    drawShapeFromPoints
} from "utils/draw";

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
const trafficLightScales = {
    x: TRAFFIC_LIGHT_SCALE,
    y: TRAFFIC_LIGHT_SCALE,
    z: TRAFFIC_LIGHT_SCALE
};

const STOP_SIGN_SCALE = 2;
const stopSignScales = {
    x: STOP_SIGN_SCALE,
    y: STOP_SIGN_SCALE,
    z: STOP_SIGN_SCALE
};

export default class Map {
    constructor() {
        loadObject(trafficLightMaterial, trafficLightObject, trafficLightScales);
        loadObject(stopSignMaterial, stopSignObject, stopSignScales);
        this.hash = -1;
        this.data = {};
        this.laneHeading = {};
        this.overlapMap = {};
        this.initialized = false;
        this.elementKindsDrawn = '';
    }

    // The result will be the all the elements in current but not in data.
    diffMapElements(elementIds, data) {
        const result = {};
        let empty = true;

        for (const kind in elementIds) {
            if (!this.shouldDrawThisElementKind(kind)) {
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
                    points, colorMapping.YELLOW, 4, 3, 3, 1, false);
            case "DOTTED_WHITE":
                return drawDashedLineFromPoints(
                    points, colorMapping.WHITE, 4, 3, 3, 1, false);
            case "SOLID_YELLOW":
                return drawSegmentsFromPoints(
                    points, colorMapping.YELLOW, 3, 1, false);
            case "SOLID_WHITE":
                return drawSegmentsFromPoints(
                    points, colorMapping.WHITE, 3, 1, false);
            case "DOUBLE_YELLOW":
                const left = drawSegmentsFromPoints(
                    points, colorMapping.YELLOW, 2, 1, false);
                const right = drawSegmentsFromPoints(
                    points.map(point =>
                        new THREE.Vector3(point.x + 0.3, point.y + 0.3, point.z)),
                    colorMapping.YELLOW, 3, 1, false);
                left.add(right);
                return left;
            case "CURB":
                return drawSegmentsFromPoints(
                    points, colorMapping.CORAL, 3, 1, false);
            default:
                return drawSegmentsFromPoints(
                    points, colorMapping.DEFAULT, 3, 1, false);
        }
    }

    addLane(lane, coordinates, scene) {
        const drewObjects = [];

        const centralLine = lane.centralCurve.segment;
        centralLine.forEach(segment => {
            const points = coordinates.applyOffsetToArray(segment.lineSegment.point);
            const centerLine =
                drawSegmentsFromPoints(points, colorMapping.GREEN, 1, 1, false);
            scene.add(centerLine);
            drewObjects.push(centerLine);
        });

        const rightLaneType = lane.rightBoundary.boundaryType[0].types[0];
        if (!lane.rightBoundary.virtual || rightLaneType !== "DOTTED_WHITE") {
            // TODO: this is a temp. fix for repeated boundary types.
            lane.rightBoundary.curve.segment.forEach((segment, index) => {
                const points = coordinates.applyOffsetToArray(segment.lineSegment.point);
                const boundary = this.addLaneMesh(rightLaneType, points);
                scene.add(boundary);
                drewObjects.push(boundary);
            });
        }

        const leftLaneType = lane.leftBoundary.boundaryType[0].types[0];
        if (!lane.leftBoundary.virtual || leftLaneType !== "DOTTED_WHITE") {
            lane.leftBoundary.curve.segment.forEach((segment, index) => {
                const points = coordinates.applyOffsetToArray(segment.lineSegment.point);
                const boundary = this.addLaneMesh(leftLaneType, points);
                scene.add(boundary);
                drewObjects.push(boundary);
            });
        }

        return drewObjects;
    }

    addRoad(road, coordinates, scene) {
        const drewObjects = [];

        road.section.forEach(section => {
            section.boundary.outerPolygon.edge.forEach(edge => {
                edge.curve.segment.forEach((segment, index) => {
                    const points = coordinates.applyOffsetToArray(segment.lineSegment.point);
                    const boundary = this.addLaneMesh("CURB", points);
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
            border, color, 2, 0, true, false, 1.0);
        scene.add(mesh);
        drewObjects.push(mesh);

        return drewObjects;
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
            border, color, 2, 0, true, false, 1.0);
        scene.add(mesh);
        drewObjects.push(mesh);

        return drewObjects;
    }

    extractOverlaps(overlaps) {
        overlaps.forEach(overlap => {
            const overlapId = overlap.id.id;

            const laneIds = [];
            const signalIds = [];
            const stopIds = [];
            overlap.object.forEach(object => {
                if (object.laneOverlapInfo) {
                    laneIds.push(object.id.id);
                }
                if (object.signalOverlapInfo) {
                    signalIds.push(object.id.id);
                }
                if (object.stopSignOverlapInfo) {
                    stopIds.push(object.id.id);
                }
            });
            // Picks overlap with one signal/stop_sign and one lane.
            // Constructs a map: overlapId -> laneId
            if (laneIds.length === 1 && (stopIds.length === 1 || signalIds.length === 1)) {
                this.overlapMap[overlapId] = laneIds[0];
            }
        });
    }

    getLaneHeading(lane) {
        const last2Points = _.takeRight(_.last(lane.centralCurve.segment).lineSegment.point, 2);
        if (last2Points.length === 2) {
            return Math.atan2(last2Points[0].y - last2Points[1].y,
                last2Points[0].x - last2Points[1].x);
        }
        return 0;
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

        let heading = undefined;
        const overlapLen = signal.overlapId.length;
        if (overlapLen > 0) {
            const overlapId = signal.overlapId[overlapLen - 1].id;
            heading = this.laneHeading[this.overlapMap[overlapId]];
        }
        if (!heading) {
            console.warn("Unable to get traffic light heading, " +
                "use orthogonal direction of StopLine.");
            heading = this.getHeadingFromStopLine(signal);
        }

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

    drawStopLine(stopLines, drewObjects, coordinates, scene) {
        stopLines.forEach(stopLine => {
            stopLine.segment.forEach(segment => {
                const points = coordinates.applyOffsetToArray(segment.lineSegment.point);
                const mesh = drawSegmentsFromPoints(points, colorMapping.PURE_WHITE, 5, 3, false);
                scene.add(mesh);
                drewObjects.push(mesh);
            });
        });
    }

    addTrafficLight(signal, coordinates, scene) {
        const drewObjects = [];
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
        this.drawStopLine(signal.stopLine, drewObjects, coordinates, scene);
        return drewObjects;
    }

    getStopSignPositionAndHeading(stopSign, coordinates) {
        let heading = undefined;
        const overlapLen = stopSign.overlapId.length;
        if (overlapLen > 0) {
            const overlapId = stopSign.overlapId[0].id;
            heading = this.laneHeading[this.overlapMap[overlapId]];
        }
        if (!heading) {
            console.warn("Unable to get stop sign heading, " +
                "use orthogonal direction of StopLine.");
            heading = this.getHeadingFromStopLine(stopSign);
        }

        if (!isNaN(heading)) {
            const stopLinePoint = stopSign.stopLine[0].segment[0].lineSegment.point[0];
            let position = new THREE.Vector3(stopLinePoint.x, stopLinePoint.y, 0);
            position = coordinates.applyOffset(position);

            return { "pos": position, "heading": heading };
        } else {
            console.error('Error loading stop sign. Unable to determine heading.');
            return null;
        }
    }

    addStopSign(stopSign, coordinates, scene) {
        const drewObjects = [];
        const posAndHeading = this.getStopSignPositionAndHeading(stopSign, coordinates);
        if (posAndHeading) {
            loadObject(stopSignMaterial, stopSignObject,
                stopSignScales,
                mesh => {
                    mesh.rotation.x = Math.PI / 2;
                    mesh.rotation.y = posAndHeading.heading + Math.PI / 2;
                    mesh.position.set(posAndHeading.pos.x, posAndHeading.pos.y, 0);
                    mesh.matrixAutoUpdate = false;
                    mesh.updateMatrix();

                    scene.add(mesh);
                    drewObjects.push(mesh);
                });
        }
        this.drawStopLine(stopSign.stopLine, drewObjects, coordinates, scene);
        return drewObjects;
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
            const drawThisKind = this.shouldDrawThisElementKind(kind);
            newData[kind] = [];
            const oldDataOfThisKind = this.data[kind];
            const currentIds = elementIds[kind];
            oldDataOfThisKind.forEach(oldData => {
                if (drawThisKind && currentIds && currentIds.includes(oldData.id.id)) {
                    newData[kind].push(oldData);
                } else {
                    if (kind !== "overlap") {
                        this.removeDrewObjects(oldData.drewObjects, scene);
                    }
                    if (kind === "lane") {
                        delete this.laneHeading[oldData.id.id];
                    }
                    if (kind === "overlap") {
                        delete this.overlapMap[oldData.id.id];
                    }
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
        // Note: drawing order matter since "stopSign" and "signal" are dependent on "overlap"
        const kinds = ["overlap", "lane", "junction", "road",
                       "clearArea", "signal", "stopSign", "crosswalk"];
        for (const kind of kinds) {
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
                            drewObjects: this.addLane(lane, coordinates, scene)
                        }));
                        this.laneHeading[lane.id.id] = this.getLaneHeading(lane);
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
                    case "overlap":
                        this.extractOverlaps(newData['overlap']);
                        this.data[kind].push(newData[kind][i]);
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
                    default:
                        this.data[kind].push(newData[kind][i]);
                        break;
                }
            }
        }
    }

    shouldDrawThisElementKind(kind) {
        // Ex: mapping 'lane' to 'showMapLane' option
        const optionName = `showMap${kind[0].toUpperCase()}${kind.slice(1)}`;

        // NOTE: return true if the option is not found
        return STORE.options[optionName] !== false;
    }

    updateIndex(hash, elementIds, scene) {
        if (STORE.hmi.inNavigationMode) {
            MAP_WS.requestRelativeMapData();
        } else {
            let newElementKindsDrawn = '';
            for (const kind of Object.keys(elementIds).sort()) {
                if (this.shouldDrawThisElementKind(kind)) {
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
