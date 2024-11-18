import iconMainStop from '../../assets/images/decision/main-stop.png';
import iconObjectStop from '../../assets/images/decision/object-stop.png';
import iconObjectFollow from '../../assets/images/decision/object-follow.png';
import iconObjectYield from '../../assets/images/decision/object-yield.png';
import iconObjectOvertake from '../../assets/images/decision/object-overtake.png';

import fenceMainStop from '../../assets/images/decision/fence-main-stop.png';
import fenceObjectStop from '../../assets/images/decision/fence-object-stop.png';
import fenceObjectFollow from '../../assets/images/decision/fence-object-follow.png';
import fenceObjectYield from '../../assets/images/decision/fence-object-yield.png';
import fenceObjectOvertake from '../../assets/images/decision/fence-object-overtake.png';

import reasonHeadVehicle from '../../assets/images/decision/head-vehicle.png';
import reasonDestination from '../../assets/images/decision/destination.png';
import reasonPedestrian from '../../assets/images/decision/pedestrian.png';
import reasonObstacle from '../../assets/images/decision/obstacle.png';
import reasonSignal from '../../assets/images/decision/signal.png';
import reasonStopSign from '../../assets/images/decision/stop-sign.png';
import reasonYieldSign from '../../assets/images/decision/yield-sign.png';
import reasonClearZone from '../../assets/images/decision/clear-zone.png';
import reasonCrosswalk from '../../assets/images/decision/crosswalk.png';
import reasonEmergency from '../../assets/images/decision/emergency.png';
import reasonNotReady from '../../assets/images/decision/not-ready.png';
import reasonPullover from '../../assets/images/decision/pullover.png';

import iconChangeLaneRight from '../../assets/images/decision/change-lane-right.png';
import iconChangeLaneLeft from '../../assets/images/decision/change-lane-left.png';

export const colorMapping = {
    YELLOW: 0xdaa520,
    WHITE: 0xcccccc,
    CORAL: 0xff7f50,
    RED: 0xff6666,
    GREEN: 0x006400,
    BLUE: 0x30a5ff,
    PURE_WHITE: 0xffffff,
    DEFAULT: 0xc0c0c0,
    MIDWAY: 0xff7f50,
    END: 0xffdab9,
    PULLOVER: 0x006aff,
    DEEP_RED: 0x8b0000,
};
export const zOffset = {
    lane: 0.04,
    crosswalk: 0.04,
    junction: 0.04,
    clearArea: 0.04,
    pncJunction: 0.04,
    stopLine: 0.08,
    speedBump: 0.08,
    parkingSpace: 0.04,
    checkpoint: 0.04,
    pullover: 0.12,
    routing: 0.6,
    prediction: 0.04,
    area: 0.04,
};

export const obstacleColorMapping = {
    PEDESTRIAN: 0xffea00,
    BICYCLE: 0x00dceb,
    VEHICLE: 0x00ff3c,
    VIRTUAL: 0x800000,
    CIPV: 0xff9966,
    DEFAULT: 0xff00fc,
    TRAFFICCONE: 0xe1601c,
    UNKNOWN: 0xa020f0,
    UNKNOWN_MOVABLE: 0xda70d6,
    UNKNOWN_UNMOVABLE: 0xff00ff,
};

export const pointCloudHeightColorMapping = {
    0.5: {
        r: 255,
        g: 0,
        b: 0,
    },
    1.0: {
        r: 255,
        g: 127,
        b: 0,
    },
    1.5: {
        r: 255,
        g: 255,
        b: 0,
    },
    2.0: {
        r: 0,
        g: 255,
        b: 0,
    },
    2.5: {
        r: 0,
        g: 0,
        b: 255,
    },
    3.0: {
        r: 75,
        g: 0,
        b: 130,
    },
    10.0: {
        r: 148,
        g: 0,
        b: 211,
    },
};

export const decisionMarkerColorMapping = {
    STOP: 0xff3030,
    FOLLOW: 0x1ad061,
    YIELD: 0xff30f7,
    OVERTAKE: 0x30a5ff,
};

export const mainDecisionStopReasonMarkerMapping = {
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
    STOP_REASON_NOT_READY: reasonNotReady,
    STOP_REASON_PULL_OVER: reasonPullover,
};

export const mainDecisionChangeLaneMarkerMapping = {
    LEFT: iconChangeLaneLeft,
    RIGHT: iconChangeLaneRight,
};

export const decisionFenceMapping = {
    STOP: fenceObjectStop,
    FOLLOW: fenceObjectFollow,
    YIELD: fenceObjectYield,
    OVERTAKE: fenceObjectOvertake,
    MAIN_STOP: fenceMainStop,
};

export const obstacleDecisionIconMapping = {
    STOP: iconObjectStop,
    FOLLOW: iconObjectFollow,
    YIELD: iconObjectYield,
    OVERTAKE: iconObjectOvertake,
    MAIN_STOP: iconMainStop,
};

export const cameraParams = {
    Default: {
        fov: 60,
        near: 1,
        far: 300,
    },
    Near: {
        fov: 60,
        near: 1,
        far: 200,
    },
    Overhead: {
        fov: 60,
        near: 1,
        far: 100,
    },
    Map: {
        fov: 70,
        near: 1,
        far: 4000,
    },
};
