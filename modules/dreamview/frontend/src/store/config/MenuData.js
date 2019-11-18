export default [
    {
        id: 'camera',
        title: 'Point of View',
        type: 'radio',
        data: {
            1: 'Default',
            2: 'Near',
            3: 'Overhead',
            4: 'Map',
            5: 'CameraView',
        },
        supportInOfflineView: true,
    },
    {
        id: 'perception',
        title: 'Perception',
        type: 'checkbox',
        data: {
            perceptionPointCloud: 'Point Cloud',
            perceptionVehicle: 'Vehicle',
            perceptionPedestrian: 'Pedestrian',
            perceptionBicycle: 'Bicycle',
            perceptionUnknownMovable: 'Unknown Movable',
            perceptionUnknownUnmovable: 'Unknown Stationary',
            perceptionUnknown: 'Unknown',
            perceptionCipv: 'Closest In-Path Vehicle',
            perceptionVelocity: 'Velocity',
            perceptionHeading: 'Heading',
            perceptionId: 'Id',
            perceptionObstacleInfo: 'Distance and Speed',
            perceptionLaneMarker: 'Lane Marker',
        },
        supportInOfflineView: true,
    }, {
        id: 'prediction',
        title: 'Prediction',
        type: 'checkbox',
        data: {
            predictionMajor: 'Major Prediction Line',
            predictionMinor: 'Minor Prediction Line',
            predictionGaussianInfo: 'Gaussian Info',
            predictionPriority: 'Priority'
        },
        supportInOfflineView: true,
    }, {
        id: 'routing',
        title: 'Routing',
        type: 'checkbox',
        data: {
            routing: 'Routing Line'
        },
        supportInOfflineView: true,
    }, {
        id: 'decision',
        title: 'Decision',
        type: 'checkbox',
        data: {
            decisionMain: 'Main Decision',
            decisionObstacle: 'Obstacle Decision',
        },
        supportInOfflineView: true,
    }, {
        id: 'planning',
        title: 'Planning',
        type: 'checkbox',
        data: {
            planningCar: 'Planning Car',
            planningTrajectory: 'Planning Trajectory',
            rssInfo: 'RSS Info'
        },
        supportInOfflineView: true,
    }, {
        id: 'position',
        title: 'Position',
        type: 'checkbox',
        data: {
            positionLocalization: 'Localization',
            positionGps: 'GPS',
            positionShadow: 'Shadow',
        },
        supportInOfflineView: true,
    }, {
        id: 'map',
        title: 'Map',
        type: 'checkbox',
        data: {
            mapCrosswalk: 'Crosswalk',
            mapClearArea: 'Clear Area',
            mapJunction: 'Junction',
            mapPncJunction: 'PNC Junction',
            mapLane: 'Lane',
            mapRoad: 'Road',
            mapSignal: 'Signal',
            mapStopSign: 'Stop Sign',
            mapYield: 'Yield Sign',
            mapSpeedBump: 'Speed Bump',
            mapParkingSpace: 'Parking Space',
            mapLaneId: 'Lane Id'
        },
        supportInOfflineView: false,
    },
];
