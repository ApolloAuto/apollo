import * as THREE from "three";
import _ from "lodash";

import { loadObject } from "utils/models";

export default class TrafficControlsBase {
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

    static loadModel(material, object, scales, position, heading, callback) {
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

    static remove(mesh, scene) {
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

    constructor(material, object, scale) {
        this.object = {};

        this.model = {
            material,
            object,
            scales: {
                x: scale,
                y: scale,
                z: scale,
            },
        };
    }

    getParams(item, coordinates) {
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
            const posAndHeading = this.getPositionAndHeading(item, coordinates);
            params.heading = posAndHeading.heading;
            params.position = posAndHeading.pos;
            params.id = item.id.id;
        }
        return params;
    }

    getPositionAndHeading(sign, coordinates) {
        // Return dummy as this should be implemented in the derived class
        return { "pos": new THREE.Vector3(0, 0, 0), "heading": 0 };
    }

    add(items, coordinates, scene) {
        if (_.isEmpty(items)) {
            return;
        }

        items.forEach((item) => {
            const { position, heading, id } = this.getParams(item, coordinates);

            TrafficControlsBase.loadModel(this.model.material, this.model.object, this.model.scales,
                position, heading, (err, mesh) => {
                    if (err) {
                        return console.error('Failed to load model:', err);
                    }

                    this.object[id] = mesh;
                    scene.add(mesh);
                });
        });
    }

    removeAll(scene) {
        if (_.isEmpty(this.object)) {
            return;
        }

        for (const id in this.object) {
            TrafficControlsBase.remove(this.object[id], scene);
        }
        this.object = {};
    }

    removeExpired(currentItemIds, scene) {
        if (_.isEmpty(this.object)) {
            return;
        }

        const currentItems = {};
        for (const id in this.object) {
            const mesh = this.object[id];
            if (!currentItemIds || !currentItemIds.includes(id)) {
                TrafficControlsBase.remove(mesh, scene);
            } else {
                currentItems[id] = mesh;
            }
        }

        this.object = currentItems;
    }
}
