import * as THREE from "three";

import STORE from "store";
import Text3D from "renderer/text3d";
import { copyProperty, hideArrayObjects } from "utils/misc";
import { drawSegmentsFromPoints, drawBox, drawArrow } from "utils/draw";
const _ = require('lodash');

const DEFAULT_HEIGHT = 1.5;
export const DEFAULT_COLOR = 0xFF00FC;
export const ObstacleColorMapping = {
        PEDESTRIAN: 0xFFEA00,
        BICYCLE: 0x00DCEB,
        VEHICLE: 0x00FF3C,
        VIRTUAL: 0x800000
};
const LINE_THICKNESS = 1.5;

export default class PerceptionObstacles {
    constructor() {
        this.textRender = new Text3D();
        this.arrows = []; // for indication of direction of moving obstacles
        this.ids = []; // for obstacle id labels
        this.cubes = []; // for obstacles with only length/width/height
        this.extrusionFaces = []; // for obstacles with polygon points
    }

    update(world, coordinates, scene) {
        // Id meshes need to be recreated everytime.
        // Each text mesh needs to be removed from the scene,
        // and its char meshes need to be hidden for reuse purpose.
        if (!_.isEmpty(this.ids)) {
            this.ids.forEach(t => {
                t.children.forEach(c => c.visible = false);
                scene.remove(t);
            });
            this.ids = [];
        }
        this.textRender.reset();

        const objects = world.object;
        if (_.isEmpty(objects)) {
            hideArrayObjects(this.arrows);
            hideArrayObjects(this.cubes);
            hideArrayObjects(this.extrusionFaces);
            return;
        }

        let arrowIdx = 0;
        let cubeIdx = 0;
        let extrusionFaceIdx = 0;
        for (let i = 0; i < objects.length; i++) {
            const obstacle = objects[i];
            if (!STORE.options['showObstacles' + _.upperFirst(_.camelCase(obstacle.type))]
                || !obstacle.positionX || !obstacle.positionY) {
                continue;
            }
            const position = coordinates.applyOffset(
                    new THREE.Vector3(obstacle.positionX,
                                      obstacle.positionY,
                                      (obstacle.height || DEFAULT_HEIGHT) / 2));
            const color = ObstacleColorMapping[obstacle.type] || DEFAULT_COLOR;
            if (STORE.options.showObstaclesVelocity && obstacle.type &&
                    obstacle.type !== 'UNKNOWN_UNMOVABLE' && obstacle.speed > 0.5) {
                const arrowMesh = this.updateArrow(position,
                        obstacle.speedHeading, color, arrowIdx++, scene);
                const scale = 1 + Math.log2(obstacle.speed);
                arrowMesh.scale.set(scale, scale, scale);
                arrowMesh.visible = true;
            }
            if (STORE.options.showObstaclesHeading) {
                const arrowMesh = this.updateArrow(position, obstacle.heading,
                        0xFFFFFF, arrowIdx++, scene);
                arrowMesh.scale.set(1, 1, 1);
                arrowMesh.visible = true;
            }
            if (STORE.options.showObstaclesId) {
                this.updateId(obstacle.id,
                        new THREE.Vector3(position.x, position.y, obstacle.height),
                        scene);
            }
            const polygon = obstacle.polygonPoint;
            if (polygon.length > 0) {
                const scale = this.updatePolygon(polygon, obstacle.height, color, coordinates,
                        extrusionFaceIdx, scene);
                extrusionFaceIdx += polygon.length;
                // arrowMesh.scale.set(scale, scale, scale);
            } else if (obstacle.length && obstacle.width && obstacle.height) {
                this.updateCube(obstacle.length, obstacle.width, obstacle.height, position,
                        obstacle.heading, color, cubeIdx++, scene);
                // arrowMesh.scale.set(obstacle.width, obstacle.length, obstacle.height);
            }
        }
        hideArrayObjects(this.arrows, arrowIdx);
        hideArrayObjects(this.cubes, cubeIdx);
        hideArrayObjects(this.extrusionFaces, extrusionFaceIdx);
    }

    updateArrow(position, heading, color, arrowIdx, scene) {
        const arrowMesh = this.getArrow(arrowIdx, scene);
        copyProperty(arrowMesh.position, position);
        arrowMesh.material.color.setHex(color);
        arrowMesh.rotation.set(0, 0, -(Math.PI / 2 - heading));
        return arrowMesh;
    }

    updateId(id, position, scene) {
        const text = this.textRender.composeText(id);
        if (text === null) {
            return;
        }
        text.position.set(position.x, position.y, position.z || 3);
        const camera = scene.getObjectByName("camera");
        if (camera !== undefined) {
            text.quaternion.copy(camera.quaternion);
        }
        text.children.forEach(c => c.visible = true);
        text.visible = true;
        text.name = "id_" + id;
        this.ids.push(text);
        scene.add(text);
    }

    updatePolygon(points, height, color, coordinates, extrusionFaceIdx, scene) {
        let edgeDistanceSum = 0;
        for (let i = 0; i < points.length; i++) {
            const faceMesh = this.getFace(extrusionFaceIdx + i, scene);
            const next = (i === points.length - 1) ? 0 : i + 1;
            const v = new THREE.Vector3(points[i].x, points[i].y, points[i].z);
            const vNext = new THREE.Vector3(points[next].x, points[next].y, points[next].z);
            const facePosition = coordinates.applyOffset(
                    new THREE.Vector2((v.x + vNext.x) / 2.0, (v.y + vNext.y) / 2.0));

            if (facePosition === null) {
                continue;
            }
            faceMesh.position.set(facePosition.x, facePosition.y, 0);
            const edgeDistance = v.distanceTo(vNext);
            edgeDistanceSum += edgeDistance;
            faceMesh.scale.set(edgeDistance, 1, height);
            faceMesh.material.color.setHex(color);
            faceMesh.rotation.set(0, 0, Math.atan2(vNext.y - v.y, vNext.x - v.x));
            faceMesh.visible = true;
        }
        return 1.0 * edgeDistanceSum / points.length;
    }

    updateCube(length, width, height, position, heading, color, cubeIdx, scene) {
        const cubeMesh = this.getCube(cubeIdx, scene);
        cubeMesh.position.set(position.x, position.y, position.z);
        cubeMesh.scale.set(length, width, height);
        cubeMesh.material.color.setHex(color);
        cubeMesh.rotation.set(0, 0, heading);
        cubeMesh.visible = true;
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

    getFace(index, scene) {
        if (index < this.extrusionFaces.length) {
            return this.extrusionFaces[index];
        }
        const extrusionFace = drawSegmentsFromPoints([
            new THREE.Vector3(-0.5, 0, 0),
            new THREE.Vector3(0.5, 0, 0),
            new THREE.Vector3(0.5, 0, 1),
            new THREE.Vector3(-0.5, 0, 1)
        ], DEFAULT_COLOR, LINE_THICKNESS);
        extrusionFace.visible = false;
        this.extrusionFaces.push(extrusionFace);
        scene.add(extrusionFace);
        return extrusionFace;
    }

    getCube(index, scene) {
        if (index < this.cubes.length) {
            return this.cubes[index];
        }
        const cubeMesh = drawBox(new THREE.Vector3(1, 1, 1), DEFAULT_COLOR, LINE_THICKNESS);
        cubeMesh.visible = false;
        this.cubes.push(cubeMesh);
        scene.add(cubeMesh);
        return cubeMesh;
    }
}
