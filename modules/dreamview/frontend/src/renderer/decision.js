import * as THREE from "three";

import STORE from "store";

import mainStopMarker from "assets/images/decision/main-stop.png";
import objectStopMarker from "assets/images/decision/object-stop.png";
import objectFollowMarker from "assets/images/decision/object-follow.png";
import objectYieldMarker from "assets/images/decision/object-yield.png";
import objectOvertakeMarker from "assets/images/decision/object-overtake.png";

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

import { copyProperty, hideArrayObjects } from "utils/misc";
import { drawImage, drawShapeFromPoints } from "utils/draw";

const _ = require('lodash');
const MarkerColorMapping = {
        STOP: 0xff7f00,
        FOLLOW: 0x00ff00,
        YIELD: 0xff00ff,
        OVERTAKE: 0x0000ff
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


export default class Decision {
    constructor() {
        this.markers = []; // for FOLLOW/STOP/YIELD/OVERTAKE decisions
        this.nudges = []; // for NUDGE decision
        this.mainDecision = this.getMainDecision(); // for main decision with reason
    }

    update(world, coordinates, scene) {
        // Nudge meshes need to be recreated everytime.
        this.nudges.forEach(n => {
            scene.remove(n);
            n.geometry.dispose();
            n.material.dispose();
        });
        this.nudges = [];

        if (!STORE.options.showDecision) {
            hideArrayObjects(this.markers);
            this.mainDecision.visible = false;
            return;
        }

        const objects = world.object;
        if (_.isEmpty(objects)) {
            return;
        }

        let markerIdx = 0;
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
                    const color = MarkerColorMapping[decisionType];
                    if (markerIdx >= this.markers.length) {
                        marker = this.getMarker(color);
                        this.markers.push(marker);
                        scene.add(marker);
                    } else {
                        marker = this.markers[markerIdx];
                    }

                    const pos = coordinates.applyOffset(new THREE.Vector3(
                            decision.positionX, decision.positionY, 0));
                    if (pos === null) {
                        continue;
                    }
                    marker.position.set(pos.x, pos.y, 0.2);
                    marker.rotation.set(Math.PI / 2, decision.heading - Math.PI / 2, 0);
                    marker.visible = true;
                    marker.objStop.visible = (decisionType === 'STOP');
                    marker.objFollow.visible = (decisionType === 'FOLLOW');
                    marker.objYield.visible = (decisionType === 'YIELD');
                    marker.objOvertake.visible = (decisionType === 'OVERTAKE');
                    marker.connect.visible = false;
                    markerIdx++;

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
                        connect.visible = true;
                    }
                } else if (decisionType === 'NUDGE') {
                    const nudge = drawShapeFromPoints(
                            coordinates.applyOffset(decision.polygonPoint),
                            new THREE.MeshBasicMaterial({color: 0xff7f00}), false, 2);
                    this.nudges.push(nudge);
                    scene.add(nudge);
                }
            }
        }
        hideArrayObjects(this.markers, markerIdx);

        // Update main decision marker.
        const mainStop = world.mainStop;
        if (_.isEmpty(mainStop)) {
            this.mainDecision.visible = false;
            return;
        }

        this.mainDecision.visible = true;
        copyProperty(this.mainDecision.position, coordinates.applyOffset(
                new THREE.Vector3(mainStop.positionX, mainStop.positionY, 0.2)));
        this.mainDecision.rotation.set(Math.PI / 2, mainStop.heading - Math.PI / 2, 0);
        const mainStopReason = _.attempt(() => mainStop.decision[0].stopReason);
        if (!_.isError(mainStopReason)) {
            let reason = null;
            for (reason in StopReasonMarkerMapping) {
                this.mainDecision[reason].visible = false;
            }
            this.mainDecision[mainStopReason].visible = true;
        }
    }

    getMainDecision() {
        const marker = new THREE.Object3D();

        const mainStop = drawImage(mainStopMarker, 2, 2.5, 0, 1, 0);
        marker.add(mainStop);

        let reason = null;
        for (reason in StopReasonMarkerMapping) {
            const reasonMarker = drawImage(StopReasonMarkerMapping[reason], 2, 2, 2, 2, 0);
            marker.add(reasonMarker);
            marker[reason] = reasonMarker;
        }

        marker.visible = false;
        return marker;
    }

    getMarker(color) {
        const marker = new THREE.Object3D();

        const objStop = drawImage(objectStopMarker, 2, 2.5, 0, 1, 0);
        marker.add(objStop);
        marker.objStop = objStop;
        const objFollow = drawImage(objectFollowMarker, 2, 2.5, 0, 1, 0);
        marker.add(objFollow);
        marker.objFollow = objFollow;
        const objYield = drawImage(objectYieldMarker, 2, 2.5, 0, 1, 0);
        marker.add(objYield);
        marker.objYield = objYield;
        const objOvertake = drawImage(objectStopMarker, 2, 2.5, 0, 1, 0);
        marker.add(objOvertake);
        marker.objOvertake = objOvertake;

        const connect = drawDashedLineFromPoints(
                [new THREE.Vector3(1, 1, 0), new THREE.Vector3(0, 0, 0)],
                color, 2, 2, 1, 30);
        marker.add(connect);
        marker.connect = connect;

        marker.visible = false;
        return marker;
    }
}