import * as THREE from "three";

import STORE from "store";

import iconMainStop from "assets/images/decision/main-stop.png";
import iconObjectStop from "assets/images/decision/object-stop.png";
import iconObjectFollow from "assets/images/decision/object-follow.png";
import iconObjectYield from "assets/images/decision/object-yield.png";
import iconObjectOvertake from "assets/images/decision/object-overtake.png";

import fenceMainStop from "assets/images/decision/fence-main-stop.png";
import fenceObjectStop from "assets/images/decision/fence-object-stop.png";
import fenceObjectFollow from "assets/images/decision/fence-object-follow.png";
import fenceObjectYield from "assets/images/decision/fence-object-yield.png";
import fenceObjectOvertake from "assets/images/decision/fence-object-overtake.png";

import reasonHeadVehicle from "assets/images/decision/head-vehicle.png";
import reasonDestination from "assets/images/decision/destination.png";
import reasonPedestrian from "assets/images/decision/pedestrian.png";
import reasonObstacle from "assets/images/decision/obstacle.png";
import reasonSignal from "assets/images/decision/signal.png";
import reasonStopSign from "assets/images/decision/stop-sign.png";
import reasonYieldSign from "assets/images/decision/yield-sign.png";
import reasonClearZone from "assets/images/decision/clear-zone.png";
import reasonCrosswalk from "assets/images/decision/crosswalk.png";
import reasonEmergency from "assets/images/decision/emergency.png";
import reasonNotReady from "assets/images/decision/not-ready.png";

import iconChangeLaneRight from "assets/images/decision/change-lane-right.png";
import iconChangeLaneLeft from "assets/images/decision/change-lane-left.png";

import { copyProperty, hideArrayObjects } from "utils/misc";
import { drawImage, drawDashedLineFromPoints, drawShapeFromPoints } from "utils/draw";

const _ = require('lodash');
const MarkerColorMapping = {
    STOP: 0xFF3030,
    FOLLOW: 0x1AD061,
    YIELD: 0xFF30F7,
    OVERTAKE: 0x30A5FF
};
const StopReasonMarkerMapping = {
    STOP_REASON_HEAD_VEHICLE: reasonHeadVehicle,
    STOP_REASON_DESTINATION: reasonDestination,
    STOP_REASON_PEDESTRIAN: reasonPedestrian,
    STOP_REASON_OBSTACLE: reasonObstacle,
    STOP_REASON_SIGNAL: reasonSignal,
    STOP_REASON_STOP_SIGN: reasonStopSign,
    STOP_REASON_YIELD_SIGN: reasonYieldSign,
    STOP_REASON_CLEAR_ZONE: reasonClearZone,
    STOP_REASON_CROSSWALK: reasonCrosswalk,
    STOP_REASON_EMERGENCY: reasonEmergency,
    STOP_REASON_NOT_READY: reasonNotReady
};

const ChangeLaneMarkerMapping = {
    LEFT: iconChangeLaneLeft,
    RIGHT: iconChangeLaneRight,
};

export default class Decision {
    constructor() {
        // for STOP/FOLLOW/YIELD/OVERTAKE decisions
        this.markers = {
            STOP: [],
            FOLLOW: [],
            YIELD: [],
            OVERTAKE: []
        };
        this.nudges = []; // for NUDGE decision

        this.mainDecision = {
            "STOP": this.getMainStopDecision(),
            "CHANGE_LANE": this.getMainChangeLaneDecision(),
        };
        this.mainDecisionAddedToScene = false;
    }

    update(world, coordinates, scene) {
        // Nudge meshes need to be recreated every time.
        this.nudges.forEach(n => {
            scene.remove(n);
            n.geometry.dispose();
            n.material.dispose();
        });
        this.nudges = [];

        this.updateMainDecision(world, coordinates, scene);
        this.updateObstacleDecision(world, coordinates, scene);
    }

    updateMainDecision(world, coordinates, scene) {
        const worldMainDecision = world.mainDecision ? world.mainDecision : world.mainStop;
        for (const type in this.mainDecision) {
            this.mainDecision[type].visible = false;
        }

        if (STORE.options.showDecisionMain && !_.isEmpty(worldMainDecision)) {
            if (!this.mainDecisionAddedToScene) {
                for (const type in this.mainDecision) {
                    scene.add(this.mainDecision[type]);
                }
                this.mainDecisionAddedToScene = true;
            }

            // un-set markers
            for (const reason in StopReasonMarkerMapping) {
                this.mainDecision["STOP"][reason].visible = false;
            }
            for (const type in ChangeLaneMarkerMapping) {
                this.mainDecision["CHANGE_LANE"][type].visible = false;
            }

            // set reasons
            const defaultPosition = coordinates.applyOffset({
                x: worldMainDecision.positionX,
                y: worldMainDecision.positionY,
                z: 0.2,
            });
            const defaultHeading = worldMainDecision.heading;
            for (const decision of worldMainDecision.decision) {
                let position = defaultPosition;
                let heading = defaultHeading;
                if (_.isNumber(decision.positionX) && _.isNumber(decision.positionY)) {
                    position = coordinates.applyOffset({
                        x: decision.positionX,
                        y: decision.positionY,
                        z: 0.2,
                    });
                }
                if (_.isNumber(decision.heading)) {
                    heading = decision.heading;
                }

                const mainStopReason = _.attempt(() => decision.stopReason);
                if (!_.isError(mainStopReason) && mainStopReason) {
                    this.mainDecision["STOP"].position.set(position.x, position.y, position.z);
                    this.mainDecision["STOP"].rotation.set(Math.PI / 2, heading - Math.PI / 2, 0);
                    this.mainDecision["STOP"][mainStopReason].visible = true;
                    this.mainDecision["STOP"].visible = true;
                }

                const changeLaneType = _.attempt(() => decision.changeLaneType);
                if (!_.isError(changeLaneType) && changeLaneType) {
                    this.mainDecision["CHANGE_LANE"].position.set(
                        position.x, position.y, position.z);
                    this.mainDecision["CHANGE_LANE"].rotation.set(
                        Math.PI / 2, heading - Math.PI / 2, 0);
                    this.mainDecision["CHANGE_LANE"][changeLaneType].visible = true;
                    this.mainDecision["CHANGE_LANE"].visible = true;
                }
            }
        }
    }

    updateObstacleDecision(world, coordinates, scene) {
        const objects = world.object;
        if (!STORE.options.showDecisionObstacle || _.isEmpty(objects)) {
            let decision = null;
            for (decision in MarkerColorMapping) {
                hideArrayObjects(this.markers[decision]);
            }
            return;
        }

        const markerIdx = { STOP: 0, FOLLOW: 0, YIELD: 0, OVERTAKE: 0 };
        for (let i = 0; i < objects.length; i++) {
            const decisions = objects[i].decision;
            if (_.isEmpty(decisions)) {
                continue;
            }
            for (let j = 0; j < decisions.length; ++j) {
                const decision = decisions[j];
                const decisionType = _.attempt(() => decision.type);
                if (_.isError(decisionType)) {
                    continue;
                }

                if (decisionType === 'STOP' || decisionType === 'FOLLOW' ||
                    decisionType === 'YIELD' || decisionType === 'OVERTAKE') {
                    // Show the specific marker.
                    let marker = null;
                    if (markerIdx[decisionType] >= this.markers[decisionType].length) {
                        marker = this.getObstacleDecision(decisionType);
                        this.markers[decisionType].push(marker);
                        scene.add(marker);
                    } else {
                        marker = this.markers[decisionType][markerIdx[decisionType]];
                    }

                    const pos = coordinates.applyOffset(new THREE.Vector3(
                        decision.positionX, decision.positionY, 0));
                    if (pos === null) {
                        continue;
                    }
                    marker.position.set(pos.x, pos.y, 0.2);
                    marker.rotation.set(Math.PI / 2, decision.heading - Math.PI / 2, 0);
                    marker.visible = true;
                    markerIdx[decisionType]++;

                    if (decisionType === 'YIELD' || decisionType === 'OVERTAKE') {
                        // Draw a dotted line to connect the marker and the obstacle.
                        const connect = marker.connect;
                        connect.geometry.vertices[0].set(
                            objects[i].positionX - decision.positionX,
                            objects[i].positionY - decision.positionY, 0);
                        connect.geometry.verticesNeedUpdate = true;
                        connect.geometry.computeLineDistances();
                        connect.geometry.lineDistancesNeedUpdate = true;
                        connect.rotation.set(Math.PI / (-2), 0,
                            Math.PI / 2 - decision.heading);
                    }
                } else if (decisionType === 'NUDGE') {
                    const nudge = drawShapeFromPoints(
                        coordinates.applyOffsetToArray(decision.polygonPoint),
                        new THREE.MeshBasicMaterial({ color: 0xff7f00 }), false, 2);
                    this.nudges.push(nudge);
                    scene.add(nudge);
                }
            }
        }
        let decision = null;
        for (decision in MarkerColorMapping) {
            hideArrayObjects(this.markers[decision], markerIdx[decision]);
        }
    }

    getMainStopDecision() {
        const marker = this.getFence("MAIN_STOP");

        for (const reason in StopReasonMarkerMapping) {
            const reasonMarker = drawImage(StopReasonMarkerMapping[reason], 1, 1, 4.2, 3.7, 0);
            marker.add(reasonMarker);
            marker[reason] = reasonMarker;
        }

        marker.visible = false;
        return marker;
    }

    getMainChangeLaneDecision() {
        const marker = this.getFence("MAIN_CHANGE_LANE");

        for (const type in ChangeLaneMarkerMapping) {
            const typeMarker = drawImage(ChangeLaneMarkerMapping[type], 1, 1, 1.0, 2.8, 0);
            marker.add(typeMarker);
            marker[type] = typeMarker;
        }

        marker.visible = false;
        return marker;
    }

    getObstacleDecision(type) {
        const marker = this.getFence(type);

        if (type === 'YIELD' || type === 'OVERTAKE') {
            const color = MarkerColorMapping[type];
            const connect = drawDashedLineFromPoints(
                [new THREE.Vector3(1, 1, 0), new THREE.Vector3(0, 0, 0)],
                color, 2, 2, 1, 30);
            marker.add(connect);
            marker.connect = connect;
        }

        marker.visible = false;
        return marker;
    }

    getFence(type) {
        let fence = null;
        let icon = null;
        const marker = new THREE.Object3D();
        switch (type) {
            case 'STOP':
                fence = drawImage(fenceObjectStop, 11.625, 3, 0, 1.5, 0);
                marker.add(fence);
                icon = drawImage(iconObjectStop, 1, 1, 3, 3.6, 0);
                marker.add(icon);
                break;
            case 'FOLLOW':
                fence = drawImage(fenceObjectFollow, 11.625, 3, 0, 1.5, 0);
                marker.add(fence);
                icon = drawImage(iconObjectFollow, 1, 1, 3, 3.6, 0);
                marker.add(icon);
                break;
            case 'YIELD':
                fence = drawImage(fenceObjectYield, 11.625, 3, 0, 1.5, 0);
                marker.add(fence);
                icon = drawImage(iconObjectYield, 1, 1, 3, 3.6, 0);
                marker.add(icon);
                break;
            case 'OVERTAKE':
                fence = drawImage(fenceObjectOvertake, 11.625, 3, 0, 1.5, 0);
                marker.add(fence);
                icon = drawImage(iconObjectOvertake, 1, 1, 3, 3.6, 0);
                marker.add(icon);
                break;
            case 'MAIN_STOP':
                fence = drawImage(fenceMainStop, 11.625, 3, 0, 1.5, 0);
                marker.add(fence);
                icon = drawImage(iconMainStop, 1, 1, 3, 3.6, 0);
                marker.add(icon);
                break;
            case 'MAIN_CHANGE_LANE':
                break;
        }
        return marker;
    }
}