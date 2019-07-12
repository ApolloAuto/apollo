import * as THREE from "three";
import _ from "lodash";

import { loadObject, loadMaterial } from "utils/models";
import trafficLightMaterial from "assets/models/traffic_light.mtl";
import trafficLightObject from "assets/models/traffic_light.obj";
import stopSignMaterial from "assets/models/stop_sign.mtl";
import stopSignObject from "assets/models/stop_sign.obj";

const TRAFFIC_LIGHT_SCALE = 2.8;
const STOP_SIGN_SCALE = 0.01;
const trafficLightScales = {
    x: TRAFFIC_LIGHT_SCALE,
    y: TRAFFIC_LIGHT_SCALE,
    z: TRAFFIC_LIGHT_SCALE,
};
const stopSignScales = {
    x: STOP_SIGN_SCALE,
    y: STOP_SIGN_SCALE,
    z: STOP_SIGN_SCALE,
};

const SUBSIGNAL_TO_INDEX = {
    GREEN: 6,
    YELLOW: 5,
    RED: 4,
};
const SUBSIGNAL_TO_LIGHT = {
    GREEN: 0x17f470,
    YELLOW: 0xcec832,
    RED: 0xff0000,
};

const EPSILON = 1e-9;

export default class TrafficControl {
    static getHeadingFromStopLine(object) {
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
    static getHeadingFromStopLineAndTrafficLightBoundary(signal) {
        // find the plane of the signal
        const boundaryPoints = signal.boundary.point;
        if (boundaryPoints.length < 3) {
            console.warn("Cannot get three points from boundary, signal_id: " + signal.id.id);
            return TrafficControl.getHeadingFromStopLine(signal);
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
            return TrafficControl.getHeadingFromStopLine(signal);
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

    static getSignalPositionAndHeading(signal, coordinates) {
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
        const heading = TrafficControl.getHeadingFromStopLineAndTrafficLightBoundary(signal);
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

    static getStopSignPositionAndHeading(stopSign, coordinates) {
        const heading = TrafficControl.getHeadingFromStopLine(stopSign);

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

    static loadMapObject(material, object, scales, position, heading, callback) {
        if (!heading || !position) {
            return callback('Invalid parameters.');
        }

        loadObject(material, object, scales, mesh => {
            mesh.rotation.x = Math.PI / 2;
            mesh.rotation.y = heading;
            mesh.position.set(position.x, position.y, 0);
            mesh.matrixAutoUpdate = false;
            mesh.updateMatrix();

            callback(null, mesh);
        });
    }

    static removeMapObject(mesh, scene) {
        if (mesh) {
            scene.remove(mesh);
            mesh.traverse(child => {
                if (child.geometry) {
                    child.geometry.dispose();
                }
                if (child.material) {
                    if (child.material.materials) {
                        child.material.materials.forEach((material) => {
                            material.dispose();
                        });
                    } else {
                        child.material.dispose();
                    }
                }
            });
        }
    }

    static getMapObjectParams(item, coordinates, getPosAndHeading) {
        const params = {};
        if (OFFLINE_PLAYBACK) {
            params.heading = item.heading;
            params.position = coordinates.applyOffset({
                x: item.x,
                y: item.y,
                z: 0,
            });
            params.id = item.id;
        } else {
            const posAndHeading = getPosAndHeading(item, coordinates);
            params.heading = posAndHeading.heading;
            params.position = posAndHeading.pos;
            params.id = item.id.id;
        }
        return params;
    }

    constructor() {
        this.hash = -1;
        this.trafficLight = {};
        this.stopSign = {};
        this.shiningSubsignals = [];

        // Prepare materials for red/yellow/green light
        loadMaterial(trafficLightMaterial, materials => {
            const [ , , DARK_GREEN, DARK_YELLOW, DARK_RED ] = materials.getAsArray();

            const getLightMaterial = (material, subsignal) => {
                const lightMaterial = material.clone();
                lightMaterial.emissive.set(SUBSIGNAL_TO_LIGHT[subsignal]);
                lightMaterial.dispose();
                return lightMaterial;
            };

            this.SUBSIGNAL_MATERIALS = {
                GREEN: {
                    DARK: DARK_GREEN,
                    LIGHT: getLightMaterial(DARK_GREEN, 'GREEN'),
                },
                YELLOW: {
                    DARK: DARK_YELLOW,
                    LIGHT: getLightMaterial(DARK_YELLOW, 'YELLOW'),
                },
                RED: {
                    DARK: DARK_RED,
                    LIGHT: getLightMaterial(DARK_RED, 'RED'),
                }
            };
        });
    }

    addTrafficLight(items, coordinates, scene) {
        if (!items || items.length === 0) {
            return;
        }

        items.forEach((item) => {
            const { position, heading, id } = TrafficControl.getMapObjectParams(
                item, coordinates, TrafficControl.getSignalPositionAndHeading);

            TrafficControl.loadMapObject(trafficLightMaterial, trafficLightObject,
                trafficLightScales, position, heading, (err, mesh) => {
                if (err) {
                    return;
                }

                this.trafficLight[id] = mesh;
                scene.add(mesh);
            });
        });
    }

    addStopSign(items, coordinates, scene) {
        if (!items || items.length === 0) {
            return;
        }

        items.forEach((item) => {
            const { position, heading, id } = TrafficControl.getMapObjectParams(
                item, coordinates, TrafficControl.getStopSignPositionAndHeading);

            TrafficControl.loadMapObject(stopSignMaterial, stopSignObject,
                stopSignScales, position, heading, (err, mesh) => {
                if (err) {
                    return;
                }

                this.stopSign[id] = mesh;
                scene.add(mesh);
            });
        });
    }

    removeTrafficLight(currentItemIds, scene) {
        if (!this.trafficLight) {
            return;
        }

        const currentItems = {};
        for (const id in this.trafficLight) {
            const mesh = this.trafficLight[id];
            if (!currentItemIds || !currentItemIds.includes(id)) {
                TrafficControl.removeMapObject(mesh, scene);
            } else {
                currentItems[id] = mesh;
            }
        }

        this.trafficLight = currentItems;
    }

    removeStopSign(currentItemIds, scene) {
        if (!this.stopSign) {
            return;
        }

        const currentItems = {};
        for (const id in this.stopSign) {
            const mesh = this.stopSign[id];
            if (!currentItemIds || !currentItemIds.includes(id)) {
                TrafficControl.removeMapObject(mesh, scene);
            } else {
                currentItems[id] = mesh;
            }
        }

        this.stopSign = currentItems;
    }

    clearTrafficLightStatus() {
        if (this.shiningSubsignals.length > 0) {
            this.shiningSubsignals.forEach((mesh) => {
                const darkMaterial = _.get(this.SUBSIGNAL_MATERIALS, `${mesh.subsignal}.DARK`);
                if (darkMaterial) {
                    mesh.material = darkMaterial;
                }
            });
            this.shiningSubsignals = [];
        }
    }

    updateTrafficLightStatus(trafficSignal) {
        if (!Array.isArray(trafficSignal)) {
            return;
        }

        this.clearTrafficLightStatus();

        if (trafficSignal && this.trafficLight) {
            const signalId2Color = {};
            trafficSignal.forEach(signal => signalId2Color[signal.id] = signal.currentSignal);

            Object.keys(this.trafficLight)
                .filter(id => id in signalId2Color)
                .forEach((id) => {
                    const mesh = this.trafficLight[id];
                    if (mesh) {
                        const subsignal = signalId2Color[id];
                        const index = SUBSIGNAL_TO_INDEX[subsignal];
                        if (index) {
                            const lightMaterial = _.get(
                                this.SUBSIGNAL_MATERIALS, `${subsignal}.LIGHT`);

                            const subsignalMesh = mesh.children[index];
                            subsignalMesh.material = lightMaterial;
                            subsignalMesh.subsignal = subsignal;

                            this.shiningSubsignals.push(subsignalMesh);
                        }
                    }
            });
        }
    }
}
