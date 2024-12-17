interface ChildLayerVisible {
    [key: string]: boolean;
}

export const layerVisible: { [key: string]: ChildLayerVisible } = {
    Perception: {
        pointCloud: true,
        vehicle: false,
        pedestrian: false,
        bicycle: false,
        unknownMovable: false,
        unknownUnmovable: false,
        unknown: false,
        cipv: false,
        obstacleVelocity: false,
        obstacleHeading: false,
        obstacleId: false,
        obstacleDistanceAndSpeed: false,
        laneMarker: false,
        lidarSensor: false,
        radarSensor: false,
        cameraSensor: false,
        v2x: false,
    },
    Prediction: {
        majorPredictionLine: false,
        minorPredictionLine: false,
        gaussianInfo: false,
        obstaclePriority: false,
        obstacleInteractiveTag: false,
    },
    Routing: {
        routingLine: false,
    },
    Decision: {
        mainDecision: false,
        obstacleDecision: false,
    },
    Planning: {
        planningCar: false,
        planningTrajectory: false,
    },
    Position: {
        localization: false,
        gps: false,
        shadow: false,
    },
    Map: {
        crosswalk: false,
        clearArea: false,
        junction: false,
        pncJunction: false,
        lane: false,
        road: false,
        signal: false,
        stopSign: false,
        yieldSign: false,
        speedBump: false,
        parkingSpace: false,
        barrierGate: false,
        area: false,
        laneId: false,
        parkingSpaceId: false,
    },
};
