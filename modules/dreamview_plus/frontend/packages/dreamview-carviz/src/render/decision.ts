import * as THREE from 'three';
import loadshIsNumber from 'lodash/isNumber';
import { disposeGroup, disposeMesh, drawImge, drawShapeFromPoints } from '../utils/common';
import { drawDashedLineFromPoints } from '../utils/line';
import {
    mainDecisionChangeLaneMarkerMapping,
    mainDecisionStopReasonMarkerMapping,
    decisionMarkerColorMapping,
    obstacleDecisionIconMapping,
    decisionFenceMapping,
} from '../constant/common';

export default class Decision {
    private fenceMeshMapping;

    private iconMeshMapping;

    private mainStopReasonMeshMapping;

    private mainChangeLaneMeshMapping;

    private mainDecisionGroups;

    private obstacleDecisionGroupsAndMeshs;

    private scene;

    private option;

    private coordinates;

    private colors;

    constructor(scene, option, coordinates, colors?) {
        this.colors = colors?.decisionMarkerColorMapping || decisionMarkerColorMapping;
        this.scene = scene;
        this.option = option;
        this.coordinates = coordinates;
        this.fenceMeshMapping = {};
        this.iconMeshMapping = {};
        this.mainChangeLaneMeshMapping = {};
        this.mainStopReasonMeshMapping = {};
        this.mainDecisionGroups = [];
        this.obstacleDecisionGroupsAndMeshs = [];

        this.initFenceMesh();
        this.initIconMesh();
        this.initMainStopReasonMesh();
        this.initMainChangeLaneMesh();
    }

    update(mainDecision, obstacles) {
        const { mainDecision: mainDecisionVisible, obstacleDecision: obstacleDecisionVisible } =
            this.option.layerOption.Decision;
        this.disposeMainDecisionMeshs();
        this.disposeObstacleDecisionMeshs();
        if (mainDecisionVisible) {
            this.updateMainDecision(mainDecision);
        }
        if (obstacleDecisionVisible) {
            this.updateObstacleDecision(obstacles);
        }
    }

    updateMainDecision(mainDecision) {
        this.disposeMainDecisionMeshs();
        if (!this.option.layerOption.Decision.mainDecision) {
            return;
        }

        if (Object.keys(mainDecision).length === 0) {
            return;
        }
        if (!this.coordinates.isInitialized()) {
            return;
        }
        const { positionX, positionY, heading, decision } = mainDecision;
        const position = this.coordinates.applyOffset(new THREE.Vector3(positionX, positionY, 0.2));
        for (let i = 0; i < decision.length; i += 1) {
            const dec = decision[i];
            const { stopReason, changeLaneType } = dec;
            const group = new THREE.Group();
            let curPosition = position;
            let curHeading = heading;
            if (loadshIsNumber(dec.positionX) && loadshIsNumber(dec.positionY)) {
                curPosition = this.coordinates.applyOffset({
                    x: dec.positionX,
                    y: dec.positionY,
                    z: 0.2,
                });
            }
            if (loadshIsNumber(dec.heading)) {
                curHeading = dec.heading;
            }
            if (stopReason) {
                if (this.mainStopReasonMeshMapping[stopReason]) {
                    const mesh = this.mainStopReasonMeshMapping[stopReason].clone();
                    mesh.position.set(4.2, 3.6, 0);
                    group.add(mesh);
                }
                if (this.fenceMeshMapping.MAIN_STOP) {
                    const fence = this.fenceMeshMapping.MAIN_STOP.clone();
                    fence.position.set(0, 1.5, 0);
                    group.add(fence);
                }

                if (this.iconMeshMapping.MAIN_STOP) {
                    const icon = this.iconMeshMapping.MAIN_STOP.clone();
                    icon.position.set(3, 3.6, 0);
                    group.add(icon);
                }
            }

            if (changeLaneType) {
                if (this.mainChangeLaneMeshMapping[changeLaneType]) {
                    const mesh = this.mainChangeLaneMeshMapping[changeLaneType].clone();
                    mesh.position.set(1.0, 2.8, 0);
                    group.add(mesh);
                }
            }
            group.position.set(curPosition.x, curPosition.y, curPosition.z);

            group.rotation.set(Math.PI / 2, curHeading - Math.PI / 2, 0);
            this.mainDecisionGroups.push(group);
            this.scene.add(group);
        }
    }

    updateObstacleDecision(obstacles) {
        this.disposeObstacleDecisionMeshs();
        if (!this.option.layerOption.Decision.obstacleDecision) {
            return;
        }
        if (!this.coordinates.isInitialized()) {
            return;
        }
        for (let i = 0; i < obstacles.length; i += 1) {
            const obstacle = obstacles[i];
            const decisions = obstacle.decision;
            if (!decisions || decisions.length === 0) {
                continue;
            }
            for (let j = 0; j < decisions.length; j += 1) {
                const decision = decisions[j];
                const { type, positionX, positionY, heading, polygonPoint } = decision;
                const position = this.coordinates.applyOffset({ x: positionX, y: positionY });
                if (!type) {
                    continue;
                }

                const group = new THREE.Group();
                if (type === 'STOP' || type === 'FOLLOW' || type === 'YIELD' || type === 'OVERTAKE') {
                    const fence = this.fenceMeshMapping[type].clone();
                    if (fence) {
                        fence.position.set(0, 1.5, 0);
                        group.add(fence);
                    }

                    const icon = this.iconMeshMapping[type].clone();
                    if (icon) {
                        icon.position.set(3, 3.6, 0);
                        group.add(icon);
                    }

                    if (type === 'YIELD' || type === 'OVERTAKE') {
                        if (!obstacle.positionX || !obstacle.positionY) {
                            continue;
                        }
                        const color = this.colors[type];
                        const points = [
                            new THREE.Vector3(obstacle.positionX - positionX, obstacle.positionY - positionY, 0),
                            new THREE.Vector3(0, 0, 0),
                        ];
                        const line = drawDashedLineFromPoints(points, {
                            color,
                            linewidth: 2,
                            dashSize: 2,
                            gapSize: 1,
                            zOffset: 30,
                            opacity: 1,
                            matrixAutoUpdate: true,
                        });
                        line.computeLineDistances();
                        line.rotation.set(Math.PI / -2, 0, Math.PI / 2 - decision.heading);
                        line.position.set(0, 1.5, 0);
                        group.add(line);
                    }
                    group.rotation.set(Math.PI / 2, heading - Math.PI / 2, 0);
                    group.position.set(position.x, position.y, 0.2);
                    this.obstacleDecisionGroupsAndMeshs.push(group);
                    this.scene.add(group);
                } else if (type === 'NUDGE') {
                    let points = polygonPoint.map((p) => new THREE.Vector3(p.x, p.y, p.z || 0));
                    points = this.coordinates.applyOffsetToArray(points);
                    const nudgeMesh = drawShapeFromPoints(points, 0xff7f00);
                    this.obstacleDecisionGroupsAndMeshs.push(nudgeMesh);
                    this.scene.add(nudgeMesh);
                }
            }
        }
    }

    initFenceMesh() {
        Object.keys(decisionFenceMapping).forEach((type) => {
            const image = decisionFenceMapping[type];
            const mesh = drawImge(image, 11.625, 3);
            this.fenceMeshMapping[type] = mesh;
        });
    }

    initIconMesh() {
        Object.keys(obstacleDecisionIconMapping).forEach((type) => {
            const image = obstacleDecisionIconMapping[type];
            const mesh = drawImge(image, 1, 1);
            this.iconMeshMapping[type] = mesh;
        });
    }

    initMainStopReasonMesh() {
        Object.keys(mainDecisionStopReasonMarkerMapping).forEach((type) => {
            const image = mainDecisionStopReasonMarkerMapping[type];
            const mesh = drawImge(image, 1, 1);
            this.mainStopReasonMeshMapping[type] = mesh;
        });
    }

    initMainChangeLaneMesh() {
        Object.keys(mainDecisionChangeLaneMarkerMapping).forEach((type) => {
            const image = mainDecisionChangeLaneMarkerMapping[type];
            const mesh = drawImge(image, 1, 1);
            this.mainChangeLaneMeshMapping[type] = mesh;
        });
    }

    disposeMainDecisionMeshs() {
        this.mainDecisionGroups.forEach((group) => {
            disposeGroup(group);
            this.scene.remove(group);
        });
        this.mainDecisionGroups = [];
    }

    disposeObstacleDecisionMeshs() {
        this.obstacleDecisionGroupsAndMeshs.forEach((object) => {
            if (object.type === 'Group') {
                disposeGroup(object);
            } else {
                disposeMesh(object);
            }
            this.scene.remove(object);
        });
        this.obstacleDecisionGroupsAndMeshs = [];
    }

    dispose() {
        this.disposeMainDecisionMeshs();
        this.disposeObstacleDecisionMeshs();
    }
}
