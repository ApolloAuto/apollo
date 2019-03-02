import * as THREE from "three";
import _ from 'lodash';

import STORE from "store";
import Text3D from "renderer/text3d";
import { copyProperty, hideArrayObjects, calculateLaneMarkerPoints } from "utils/misc";
import { drawSegmentsFromPoints, drawDashedLineFromPoints,
         drawBox, drawDashedBox, drawArrow, drawImage } from "utils/draw";

import iconObjectYield from "assets/images/decision/object-yield.png";

const DEFAULT_HEIGHT = 1.5;
export const DEFAULT_COLOR = 0xFF00FC;
export const ObstacleColorMapping = {
        PEDESTRIAN: 0xFFEA00,
        BICYCLE: 0x00DCEB,
        VEHICLE: 0x00FF3C,
        VIRTUAL: 0x800000,
        CIPV: 0xFF9966
};
const LINE_THICKNESS = 1.5;

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
    }

    update(world, coordinates, scene, isBirdView) {
        this.updateObjects(world, coordinates, scene, isBirdView);
        this.updateLaneMarkers(world, coordinates, scene);
    }

    updateObjects(world, coordinates, scene, isBirdView) {
        // Id meshes need to be recreated every time.
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
            hideArrayObjects(this.solidCubes);
            hideArrayObjects(this.dashedCubes);
            hideArrayObjects(this.extrusionSolidFaces);
            hideArrayObjects(this.extrusionDashedFaces);
            hideArrayObjects(this.icons);
            hideArrayObjects(this.trafficCones);
            return;
        }

        const adc = coordinates.applyOffset({
            x: world.autoDrivingCar.positionX,
            y: world.autoDrivingCar.positionY,
        });
        adc.heading = world.autoDrivingCar.heading;

        let arrowIdx = 0;
        let cubeIdx = 0;
        let extrusionFaceIdx = 0;
        let iconIdx = 0;
        let trafficConeIdx = 0;
        for (let i = 0; i < objects.length; i++) {
            const obstacle = objects[i];
            if (!STORE.options['showObstacles' + _.upperFirst(_.camelCase(obstacle.type))]
                || !_.isNumber(obstacle.positionX) || !_.isNumber(obstacle.positionY)) {
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

            this.updateTexts(adc, obstacle, position, scene, isBirdView);

            // get the confidence and validate its range
            let confidence = obstacle.confidence;
            confidence = Math.max(0.0, confidence);
            confidence = Math.min(1.0, confidence);
            const polygon = obstacle.polygonPoint;
            if (obstacle.subType === "ST_TRAFFICCONE") {
                this.updateTrafficCone(position, trafficConeIdx, scene);
                trafficConeIdx++;
            } else if (polygon !== undefined && polygon.length > 0) {
                this.updatePolygon(polygon, obstacle.height, color, coordinates, confidence,
                        extrusionFaceIdx, scene);
                extrusionFaceIdx += polygon.length;
            } else if (obstacle.length && obstacle.width && obstacle.height) {
                this.updateCube(obstacle.length, obstacle.width, obstacle.height, position,
                        obstacle.heading, color, confidence, cubeIdx++, scene);
            }

            // draw a yield sign to indicate ADC is yielding to this obstacle
            if (obstacle.yieldedObstacle) {
                const iconPosition = {
                    x: position.x,
                    y: position.y,
                    z: position.z + obstacle.height + 0.5,
                };
                this.updateIcon(iconPosition, world.autoDrivingCar.heading, iconIdx, scene);
                iconIdx++;
            }
        }
        hideArrayObjects(this.arrows, arrowIdx);
        hideArrayObjects(this.solidCubes, cubeIdx);
        hideArrayObjects(this.dashedCubes, cubeIdx);
        hideArrayObjects(this.extrusionSolidFaces, extrusionFaceIdx);
        hideArrayObjects(this.extrusionDashedFaces, extrusionFaceIdx);
        hideArrayObjects(this.icons, iconIdx);
        hideArrayObjects(this.trafficCones, trafficConeIdx);
    }

    updateArrow(position, heading, color, arrowIdx, scene) {
        const arrowMesh = this.getArrow(arrowIdx, scene);
        copyProperty(arrowMesh.position, position);
        arrowMesh.material.color.setHex(color);
        arrowMesh.rotation.set(0, 0, -(Math.PI / 2 - heading));
        return arrowMesh;
    }

    updateTexts(adc, obstacle, obstaclePosition, scene, isBirdView) {
        const initPosition = {
            x: obstaclePosition.x,
            y: obstaclePosition.y,
            z: obstacle.height || 3
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
            if (priority && priority !== "NORMAL") {
                const textPosition = {
                    x: initPosition.x + (lineCount * deltaX),
                    y: initPosition.y + (lineCount * deltaY),
                    z: initPosition.z + (lineCount * deltaZ),
                };
                this.drawTexts(priority, textPosition, scene);
            }
        }
    }

    updatePolygon(points, height, color, coordinates, confidence, extrusionFaceIdx, scene) {
        for (let i = 0; i < points.length; i++) {
            // Get cached face mesh.
            const solidFaceMesh = this.getFace(extrusionFaceIdx + i, scene, true);
            const dashedFaceMesh = this.getFace(extrusionFaceIdx + i, scene, false);

            // Get the adjacent point.
            const next = (i === points.length - 1) ? 0 : i + 1;
            const v = new THREE.Vector3(points[i].x, points[i].y, points[i].z);
            const vNext = new THREE.Vector3(points[next].x, points[next].y, points[next].z);

            // Set position.
            const facePosition = coordinates.applyOffset(
                    new THREE.Vector2((v.x + vNext.x) / 2.0, (v.y + vNext.y) / 2.0));
            if (facePosition === null) {
                continue;
            }
            solidFaceMesh.position.set(facePosition.x, facePosition.y, 0);
            dashedFaceMesh.position.set(facePosition.x, facePosition.y, height*confidence);

            // Set face scale.
            const edgeDistance = v.distanceTo(vNext);
            if (edgeDistance === 0) {
                console.warn("Cannot display obstacle with an edge length 0!");
                continue;
            }
            solidFaceMesh.scale.set(edgeDistance, 1, height*confidence);
            dashedFaceMesh.scale.set(edgeDistance, 1, height*(1 - confidence));

            solidFaceMesh.material.color.setHex(color);
            solidFaceMesh.rotation.set(0, 0, Math.atan2(vNext.y - v.y, vNext.x - v.x));
            solidFaceMesh.visible = (confidence !== 0.0);
            dashedFaceMesh.material.color.setHex(color);
            dashedFaceMesh.rotation.set(0, 0, Math.atan2(vNext.y - v.y, vNext.x - v.x));
            dashedFaceMesh.visible = (confidence !== 1.0);
        }
    }

    updateCube(length, width, height, position, heading, color, confidence, cubeIdx, scene) {
        if (confidence > 0) {
            const solidCubeMesh = this.getCube(cubeIdx, scene, true);
            solidCubeMesh.position.set(position.x, position.y, position.z+height*(confidence-1)/2 );
            solidCubeMesh.scale.set(length, width, height*confidence);
            solidCubeMesh.material.color.setHex(color);
            solidCubeMesh.rotation.set(0, 0, heading);
            solidCubeMesh.visible = true;
        }

        if (confidence < 1) {
            const dashedCubeMesh = this.getCube(cubeIdx, scene, false);
            dashedCubeMesh.position.set(position.x, position.y, position.z+height*confidence/2 );
            dashedCubeMesh.scale.set(length, width, height*(1-confidence));
            dashedCubeMesh.material.color.setHex(color);
            dashedCubeMesh.rotation.set(0, 0, heading);
            dashedCubeMesh.visible = true;
        }
    }

    updateIcon(position, heading, iconIdx, scene) {
        const icon = this.getIcon(iconIdx, scene);
        copyProperty(icon.position, position);
        icon.rotation.set(Math.PI / 2, heading - Math.PI / 2, 0);
        icon.visible = true;
    }

    updateTrafficCone(position, coneIdx, scene) {
        const cone = this.getTrafficCone(coneIdx, scene);
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

    getFace(index, scene, solid = true) {
        const extrusionFaces = solid ? this.extrusionSolidFaces : this.extrusionDashedFaces;
        if (index < extrusionFaces.length) {
            return extrusionFaces[index];
        }

        const points = [
            new THREE.Vector3(-0.5, 0, 0),
            new THREE.Vector3(0.5, 0, 0),
            new THREE.Vector3(0.5, 0, 1),
            new THREE.Vector3(-0.5, 0, 1)
        ];
        const extrusionFace = solid
            ? drawSegmentsFromPoints(points, DEFAULT_COLOR, LINE_THICKNESS)
            : drawDashedLineFromPoints(points, DEFAULT_COLOR, LINE_THICKNESS, 0.1, 0.1);
        extrusionFace.visible = false;
        extrusionFaces.push(extrusionFace);
        scene.add(extrusionFace);
        return extrusionFace;
    }

    getCube(index, scene, solid = true) {
        const cubes = solid ? this.solidCubes : this.dashedCubes;
        if (index < cubes.length) {
            return cubes[index];
        }
        const cubeSize = new THREE.Vector3(1, 1, 1);
        const cubeMesh = solid
            ? drawBox(cubeSize, DEFAULT_COLOR, LINE_THICKNESS)
            : drawDashedBox(cubeSize, DEFAULT_COLOR, LINE_THICKNESS, 0.1, 0.1);
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

    drawTexts(content, position, scene) {
        const text = this.textRender.drawText(content, scene);
        if (text) {
            text.position.set(position.x, position.y, position.z);
            this.ids.push(text);
            scene.add(text);
        }
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
                    const offsetPoints = absolutePoints.map((point) => {
                        return coordinates.applyOffset(point);
                    });
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
        cone.position.set(0, 0, height/2);
        this.trafficCones.push(cone);
        scene.add(cone);

        return cone;
    }
}
