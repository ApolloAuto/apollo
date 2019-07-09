import * as THREE from "three";

import { loadObject } from "utils/models";
import trafficLightMaterial from "assets/models/traffic_light.mtl";
import trafficLightObject from "assets/models/traffic_light.obj";
import stopSignMaterial from "assets/models/stop_sign.mtl";
import stopSignObject from "assets/models/stop_sign.obj";

const TRAFFIC_LIGHT_SCALE = 0.009;
const trafficLightScales = {
    x: TRAFFIC_LIGHT_SCALE,
    y: TRAFFIC_LIGHT_SCALE,
    z: TRAFFIC_LIGHT_SCALE
};

const STOP_SIGN_SCALE = 0.01;
const stopSignScales = {
    x: STOP_SIGN_SCALE,
    y: STOP_SIGN_SCALE,
    z: STOP_SIGN_SCALE
};

const EPSILON = 1e-9;
const TRAFFIC_LIGHT = 'signal';
const STOP_SIGN = 'stopSign';

export default class TrafficControl {
    constructor() {
        this.hash = -1;
        this.trafficControl = {
            [TRAFFIC_LIGHT]: {},
            [STOP_SIGN]: {},
        };
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

    getStopSignPositionAndHeading(stopSign, coordinates) {
        const heading = this.getHeadingFromStopLine(stopSign);

        if (!isNaN(heading)) {
            const stopLinePoint = _.last(stopSign.stopLine[0].segment[0].lineSegment.point);
            let position = new THREE.Vector3(stopLinePoint.x, stopLinePoint.y, 0);
            position = coordinates.applyOffset(position);

            return { "pos": position, "heading": heading - Math.PI / 2 };
        } else {
            console.error('Error loading stop sign. Unable to determine heading.');
            return null;
        }
    }

    addTrafficControl(items, coordinates, scene) {
        if (!items || items.length === 0) {
            return;
        }

        items.forEach((item) => {
            let material = null;
            let object = null;
            let scales = null;
            let heading = null;
            let position = null;
            let posAndHeading = null;
            switch (item.type) {
                case TRAFFIC_LIGHT:
                    material = trafficLightMaterial;
                    object = trafficLightObject;
                    scales = trafficLightScales;
                    posAndHeading = !OFFLINE_PLAYBACK
                        && this.getSignalPositionAndHeading(item, coordinates);
                    break;
                case STOP_SIGN:
                    material = stopSignMaterial;
                    object = stopSignObject;
                    scales = stopSignScales;
                    posAndHeading = !OFFLINE_PLAYBACK
                        && this.getStopSignPositionAndHeading(item, coordinates);
                    break;
                default:
                    break;
            }
            if (OFFLINE_PLAYBACK) {
                heading = item.heading;
                position = coordinates.applyOffset({
                    x: item.x,
                    y: item.y,
                    z: 0,
                });
            } else {
                heading = posAndHeading.heading;
                position = posAndHeading.pos;
            }

            if (heading && position) {
                loadObject(material, object, scales, mesh => {
                    mesh.rotation.x = Math.PI / 2;
                    mesh.rotation.y = heading;
                    mesh.position.set(position.x, position.y, 0);
                    mesh.matrixAutoUpdate = false;
                    mesh.updateMatrix();
                    scene.add(mesh);

                    const id = OFFLINE_PLAYBACK ? item.id : item.id.id;
                    this.trafficControl[item.type][id] = mesh;
                });
            }
        });
    }

    removeTrafficControl(currentItemIds, scene) {
        Object.keys(currentItemIds)
            .filter(type => [TRAFFIC_LIGHT, STOP_SIGN].includes(type))
            .forEach((type) => {
                const currentItems = {};
                Object.keys(this.trafficControl[type]).forEach((id) => {
                    const mesh = this.trafficControl[type][id];
                    if (!currentItemIds[type].includes(id)) {
                        if (mesh) {
                            scene.remove(mesh);
                            if (mesh.geometry) {
                                mesh.geometry.dispose();
                            }
                            if (mesh.material) {
                                mesh.material.dispose();
                            }
                        }
                    } else {
                        currentItems[id] = mesh;
                    }
                });
                this.trafficControl[type] = currentItems;
            });
    }

    clearTrafficLightStatus() {
        if (this.trafficControl[TRAFFIC_LIGHT]) {
            Object.values(this.trafficControl[TRAFFIC_LIGHT]).forEach((mesh) => {
                if (mesh) {
                    // clear all subsignals' status
                    [4, 5, 6].forEach((index) => {
                        mesh.children[index].material.emissive.set(0x000000);
                    });
                }
            });
        }
    }

    updateTrafficLightStatus(trafficSignal) {
        if (!Array.isArray(trafficSignal)) {
            return;
        }

        this.clearTrafficLightStatus();

        if (trafficSignal && this.trafficControl[TRAFFIC_LIGHT]) {
            const signalId2Color = {};
            trafficSignal.forEach(signal => signalId2Color[signal.id] = signal.currentSignal);

            Object.keys(this.trafficControl[TRAFFIC_LIGHT])
                .filter(id => id in signalId2Color)
                .forEach((id) => {
                    const mesh = this.trafficControl[TRAFFIC_LIGHT][id];
                    if (mesh) {
                        switch (signalId2Color[id]) {
                            case 'GREEN':
                                mesh.children[4].material.emissive.set(0x17f470);
                                break;
                            case 'YELLOW':
                                mesh.children[5].material.emissive.set(0xcec832);
                                break;
                            case 'RED':
                                mesh.children[6].material.emissive.set(0xff0000);
                                break;
                            default:
                                break;
                        }
                    }
            });
        }
    }
}