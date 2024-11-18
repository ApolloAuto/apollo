interface ChildLayerVisible {
    [key: string]: boolean;
}

export const layerVisible: { [key: string]: ChildLayerVisible } = {
    Perception: {
        pointCloud: true,
        vehicle: true,
        pedestrian: true,
        bicycle: true,
        unknownMovable: true,
        unknownUnmovable: true,
        unknown: true,
        cipv: true,
        obstacleVelocity: true,
        obstacleHeading: true,
        obstacleId: true,
        obstacleDistanceAndSpeed: true,
        laneMarker: false,
        lidarSensor: false,
        radarSensor: false,
        cameraSensor: false,
        v2x: false,
        polygon: true,
        boundingbox: false,
    },
    Prediction: {
        majorPredictionLine: true,
        minorPredictionLine: true,
        gaussianInfo: false,
        obstaclePriority: true,
        obstacleInteractiveTag: true,
    },
    Routing: {
        routingLine: true,
    },
    Decision: {
        mainDecision: true,
        obstacleDecision: true,
    },
    Planning: {
        planningCar: false,
        planningTrajectory: true,
    },
    Position: {
        localization: true,
        gps: false,
        shadow: false,
    },
    Map: {
        crosswalk: true,
        clearArea: true,
        junction: true,
        pncJunction: true,
        lane: true,
        road: false,
        signal: true,
        stopSign: true,
        yieldSign: true,
        speedBump: true,
        parkingSpace: true,
        barrierGate: true,
        area: true,
        laneId: false,
        parkingSpaceId: false,
    },
};
