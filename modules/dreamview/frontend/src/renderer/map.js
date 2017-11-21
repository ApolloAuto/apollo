import * as THREE from "three";
import WS from "store/websocket";

import { drawSegmentsFromPoints,
         drawDashedLineFromPoints,
         drawShapeFromPoints } from "utils/draw";

import trafficLightMaterial from "assets/models/traffic_light.mtl";
import trafficLightObject from "assets/models/traffic_light.obj";
import { loadObject } from "utils/models";

const colorMapping = {
    YELLOW: 0XDAA520,
    WHITE: 0xCCCCCC,
    CORAL: 0xFF7F50,
    RED: 0xFF6666,
    GREEN: 0x006400,
    PURE_WHITE: 0xFFFFFF,
    DEFAULT: 0xC0C0C0
};

const scale = 0.006;
const scales = { x: scale, y: scale, z: scale};

// The result will be the all the elements in current but not in data.
function diffMapElements(elementIds, data) {
    const result = {};
    let empty = true;

    for (const kind in elementIds) {
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

function addLaneMesh(laneType, points) {
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

function addLane(lane, coordinates, scene) {
    const drewObjects = [];

    const centralLine = lane.centralCurve.segment;
    centralLine.forEach(segment => {
        const points = coordinates.applyOffsetToArray(segment.lineSegment.point);
        const centerLine = drawSegmentsFromPoints(
            points, colorMapping.GREEN, 1, 1, false);
        scene.add(centerLine);
        drewObjects.push(centerLine);
    });

    // TODO: this is a temp. fix for repeated boundary types.
    const rightLaneType = lane.rightBoundary.boundaryType[0].types[0];
    lane.rightBoundary.curve.segment.forEach((segment, index) => {
        const points = coordinates.applyOffsetToArray(segment.lineSegment.point);
        const boundary = addLaneMesh(rightLaneType, points);
        scene.add(boundary);
        drewObjects.push(boundary);
    });

    const leftLaneType = lane.leftBoundary.boundaryType[0].types[0];
    lane.leftBoundary.curve.segment.forEach((segment, index) => {
        const points = coordinates.applyOffsetToArray(segment.lineSegment.point);
        const boundary = addLaneMesh(leftLaneType, points);
        scene.add(boundary);
        drewObjects.push(boundary);
    });

    return drewObjects;
}

function addCrossWalk(crosswalk, coordinates, scene) {
    const drewObjects = [];

    const border = coordinates.applyOffsetToArray(crosswalk.polygon.point);
    border.push(border[0]);

    const crosswalkMaterial = new THREE.MeshBasicMaterial({
        color: colorMapping.PURE_WHITE,
        transparent: true,
        opacity: .15
    });

    const crosswalkShape = drawShapeFromPoints(border, crosswalkMaterial, false, 3, false);
    scene.add(crosswalkShape);
    drewObjects.push(crosswalkShape);

    const mesh = drawSegmentsFromPoints(
        border, colorMapping.PURE_WHITE, 2, 0, true, false, 1.0);
    scene.add(mesh);
    drewObjects.push(mesh);

    return drewObjects;
}

function extractOverlaps(overlaps) {
    // get overlaps with only one lane
    const relevantOverlaps = _.filter(overlaps, o => {
        const elementIsLane = _.map(o.object, overlapElement => {
            return overlapElement.laneOverlapInfo !== undefined;
        });
        return _.countBy(elementIsLane)[true] === 1;
    });
    for (let i = 0; i < relevantOverlaps.length; i++) {
        relevantOverlaps[i].object = _.sortBy(
            relevantOverlaps[i].object, obj => obj.laneOverlapInfo === undefined);
    }

    // construct overlap map with relevant overlaps
    const keys = _.map(relevantOverlaps, overlap => overlap.id.id);
    const values = _.map(relevantOverlaps, overlap => {
        return _.join(_.map(overlap.object, o => o.id.id), '_and_');
    });
    return _.zipObject(keys, values);
}

function getLaneHeading(lane) {
    const last2Points = _.takeRight(_.last(lane.centralCurve.segment).lineSegment.point, 2);
    let res = 0;
    if (last2Points.length === 2) {
        res = Math.atan2(last2Points[0].y - last2Points[1].y, last2Points[0].x - last2Points[1].x);
    }
    return res;
}

function getSignalPositionAndHeading(signal, overlapMap, laneHeading, coordinates) {
    let locations = _.pickBy(
      _.mapValues(signal.subsignal, obj => obj.location), v => !_.isEmpty(v));
    if (_.isEmpty(locations)) {
        locations = _.attempt(() => signal.boundary.point);
    }
    if (_.isError(locations) || locations === undefined) {
        return null;
    }

    let heading = _.attempt(() => laneHeading[_.first(
        overlapMap[_.last(signal.overlapId).id].split('_and_')
    )]);
    if (_.isError(heading) || heading === undefined) {
        console.warn("Unable to get traffic light heading, use orthogonal direction of StopLine.");
        const stopLine = signal.stopLine[0].segment[0].lineSegment.point;
        const len = stopLine.length;
        if (len >= 2) {
            const stopLineDirection = Math.atan2(stopLine[len - 1].y - stopLine[0].y,
                                                 stopLine[len - 1].x - stopLine[0].x);
            heading = Math.PI * 1.5 + stopLineDirection;
        }
    }

    if (!isNaN(heading)) {
        let position = new THREE.Vector3(0, 0, 0);
        position.x = _.meanBy(_.values(locations), l => l.x);
        position.y = _.meanBy(_.values(locations), l => l.y);
        position = coordinates.applyOffset(position);
        return [position, heading];
    } else {
        console.error('Error loading traffic light. Unable to determine heading.');
        return null;
    }
}

function drawStopLine(stopLines, drewObjects, coordinates, scene) {
    stopLines.forEach(stopLine => {
        _.each(stopLine.segment, segment => {
            const points = coordinates.applyOffsetToArray(segment.lineSegment.point);
            const mesh = drawSegmentsFromPoints(points, colorMapping.PURE_WHITE, 5, 3, false);
            scene.add(mesh);
            drewObjects.push(mesh);
        });
    });
}

function addTrafficLight(signal, overlapMap, laneHeading, coordinates, scene) {
    const drewObjects = [];
    const posAndHeadings = [];
    const posAndHeading = getSignalPositionAndHeading(signal, overlapMap, laneHeading, coordinates);
    if (posAndHeading) {
        loadObject(trafficLightMaterial, trafficLightObject,
            scales,
            mesh => {
                mesh.rotation.x = Math.PI / 2;
                mesh.rotation.y = posAndHeading[1];
                mesh.position.set(posAndHeading[0].x, posAndHeading[0].y, 0);
                mesh.matrixAutoUpdate = false;
                mesh.updateMatrix();

                scene.add(mesh);
                drewObjects.push(mesh);
            });
    }
    drawStopLine(signal.stopLine, drewObjects, coordinates, scene);
    return drewObjects;
}

export default class Map {
    constructor() {
        loadObject(trafficLightMaterial, trafficLightObject, scales);
        this.hash = -1;
        this.data = {};
        this.laneHeading = {};
        this.initialized = false;
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

    removeExpiredElements(elementIds, scene) {
        const newData = {};

        for (const kind in this.data) {
            newData[kind] = [];
            const oldDataOfThisKind = this.data[kind];
            const currentIds = elementIds[kind];
            oldDataOfThisKind.forEach(oldData => {
                if (currentIds && currentIds.includes(oldData.id.id)) {
                    newData[kind].push(oldData);
                } else {
                    this.removeDrewObjects(oldData.drewObjects, scene);
                    if (kind === 'lane') {
                        delete this.laneHeading[oldData.id.id];
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
        for (const kind in newData) {
            if (!this.data[kind]) {
                this.data[kind] = [];
            }
            for (let i = 0; i < newData[kind].length; ++i) {
                switch (kind) {
                    case "lane":
                        const lane = newData[kind][i];
                        this.data[kind].push(Object.assign(newData[kind][i], {
                            drewObjects: addLane(lane, coordinates, scene)
                        }));
                        this.laneHeading[lane.id.id] = getLaneHeading(lane);
                        break;
                    case "crosswalk":
                        this.data[kind].push(Object.assign(newData[kind][i], {
                            drewObjects: addCrossWalk(
                                newData[kind][i], coordinates, scene)
                        }));
                        break;
                    case "signal":
                        const overlapMap = extractOverlaps(newData['overlap']);
                        this.data[kind].push(Object.assign(newData[kind][i], {
                            drewObjects: addTrafficLight(newData[kind][i],
                                overlapMap, this.laneHeading, coordinates, scene)
                        }));
                        break;
                    default:
                        this.data[kind].push(newData[kind][i]);
                        break;
                }
            }
        }
    }

    updateIndex(hash, elementIds, scene) {
        if (hash !== this.hash) {
            this.hash = hash;
            const diff = diffMapElements(elementIds, this.data);
            this.removeExpiredElements(elementIds, scene);
            if (!_.isEmpty(diff) || !this.initialized) {
                WS.requestMapData(diff);
                this.initialized = true;
            }
        }
    }
}
