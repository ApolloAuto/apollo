export default [
    {
        id: 'camera',
        title: 'Point of View',
        type: 'radio',
        data: {
            1: 'Default',
            2: 'Near',
            3: 'Overhead',
            4: 'Map'
        }
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
            perceptionVelocity: 'Velocity',
            perceptionHeading: 'Heading',
            perceptionId: 'Id'
        }
    }, {
        id: 'prediction',
        title: 'Prediction',
        type: 'checkbox',
        data: {
            predictionMajor: 'Major Prediction Line',
            predictionMinor: 'Minor Prediction Line'
        }
    }, {
        id: 'routing',
        title: 'Routing',
        type: 'checkbox',
        data: {
            routing: 'Routing Line'
        }
    }, {
        id: 'decision',
        title: 'Decision',
        type: 'checkbox',
        data: {
            decisionMain: 'Main Decision',
            decisionObstacle: 'Obstacle Decision',
        }
    }, {
        id: 'planning',
        title: 'Planning',
        type: 'checkbox',
        data: {
            planningCar: 'Planning Car',
            planningLine: 'Planning Line',
            planningQpOptimizer: 'Qp Optimizer Line',
            planningDpOptimizer: 'Dp Optimizer Line',
            planningReference: 'Reference Line',
        }
    }, {
        id: 'position',
        title: 'Position',
        type: 'checkbox',
        data: {
            positionLocalization: 'Localization',
            positionGps: 'GPS',
        }
    },
];
