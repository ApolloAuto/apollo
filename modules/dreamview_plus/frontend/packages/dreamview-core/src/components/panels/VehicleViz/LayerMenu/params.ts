import { LocalStorage, KEY_MANAGER } from '@dreamview/dreamview-core/src/util/storageManager';

export interface subMenuParams {
    [key: string]: {
        defaultVisible: boolean;
        currentVisible: boolean;
        vizKey: string;
    };
}

export const layerMenuParams: { [key: string]: subMenuParams } = {
    Perception: {
        polygon: {
            defaultVisible: true,
            currentVisible: true,
            vizKey: 'polygon',
        },
        boundingbox: {
            defaultVisible: false,
            currentVisible: false,
            vizKey: 'boundingbox',
        },

        pointCloud: {
            defaultVisible: false,
            currentVisible: false,
            vizKey: 'pointCloud',
        },
        unknownMovable: {
            defaultVisible: true,
            currentVisible: true,
            vizKey: 'unknownMovable',
        },
        vehicle: {
            defaultVisible: true,
            currentVisible: true,
            vizKey: 'vehicle',
        },
        unknownStationary: {
            defaultVisible: true,
            currentVisible: true,
            vizKey: 'unknownUnmovable',
        },
        pedestrian: {
            defaultVisible: true,
            currentVisible: true,
            vizKey: 'pedestrian',
        },
        unknown: {
            defaultVisible: true,
            currentVisible: true,
            vizKey: 'unknown',
        },
        bicycle: {
            defaultVisible: true,
            currentVisible: true,
            vizKey: 'bicycle',
        },
        cipv: {
            defaultVisible: true,
            currentVisible: true,
            vizKey: 'cipv',
        },
        velocity: {
            defaultVisible: true,
            currentVisible: true,
            vizKey: 'obstacleVelocity',
        },
        heading: {
            defaultVisible: true,
            currentVisible: true,
            vizKey: 'obstacleHeading',
        },
        id: {
            defaultVisible: true,
            currentVisible: true,
            vizKey: 'obstacleId',
        },
        distanceAndSpeed: {
            defaultVisible: true,
            currentVisible: true,
            vizKey: 'obstacleDistanceAndSpeed',
        },
        laneMarker: {
            defaultVisible: false,
            currentVisible: false,
            vizKey: 'laneMarker',
        },
        lidarSensor: {
            defaultVisible: false,
            currentVisible: false,
            vizKey: 'lidarSensor',
        },
        radarSensor: {
            defaultVisible: false,
            currentVisible: false,
            vizKey: 'radarSensor',
        },
        cameraSensor: {
            defaultVisible: false,
            currentVisible: false,
            vizKey: 'cameraSensor',
        },
        v2x: {
            defaultVisible: false,
            currentVisible: false,
            vizKey: 'v2x',
        },
    },
    Prediction: {
        priority: {
            defaultVisible: true,
            currentVisible: true,
            vizKey: 'obstaclePriority',
        },
        majorPredictionLine: {
            defaultVisible: true,
            currentVisible: true,
            vizKey: 'majorPredictionLine',
        },
        gaussianInfo: {
            defaultVisible: false,
            currentVisible: false,
            vizKey: 'gaussianInfo',
        },
        minorPredictionLine: {
            defaultVisible: true,
            currentVisible: true,
            vizKey: 'minorPredictionLine',
        },
        interactiveTag: {
            defaultVisible: true,
            currentVisible: true,
            vizKey: 'obstacleInteractiveTag',
        },
    },
    Routing: {
        routingLine: {
            defaultVisible: true,
            currentVisible: true,
            vizKey: 'routingLine',
        },
    },
    Decision: {
        mainDecision: {
            defaultVisible: true,
            currentVisible: true,
            vizKey: 'mainDecision',
        },
        obstacleDecision: {
            defaultVisible: true,
            currentVisible: true,
            vizKey: 'obstacleDecision',
        },
    },
    Planning: {
        planningCar: {
            defaultVisible: false,
            currentVisible: false,
            vizKey: 'planningCar',
        },
        planningTrajectoryLine: {
            defaultVisible: true,
            currentVisible: true,
            vizKey: 'planningTrajectoryLine',
        },
        planningReferenceLine: {
            defaultVisible: false,
            currentVisible: false,
            vizKey: 'PlanningReferenceLine',
        },
        planningBoundaryLine: {
            defaultVisible: false,
            currentVisible: false,
            vizKey: 'PlanningBoundaryLine',
        },
        // 'RSS Info': {
        //     defaultVisible: false,
        //     currentVisible: false,
        //     vizKey: 'pointCloud',
        // },
    },
    Position: {
        localization: {
            defaultVisible: true,
            currentVisible: true,
            vizKey: 'localization',
        },
        gps: {
            defaultVisible: false,
            currentVisible: false,
            vizKey: 'gps',
        },
        shadow: {
            defaultVisible: false,
            currentVisible: false,
            vizKey: 'shadow',
        },
    },
    Map: {
        crosswalk: {
            defaultVisible: true,
            currentVisible: true,
            vizKey: 'crosswalk',
        },
        clearArea: {
            defaultVisible: true,
            currentVisible: true,
            vizKey: 'clearArea',
        },
        junction: {
            defaultVisible: false,
            currentVisible: false,
            vizKey: 'junction',
        },
        pncJunction: {
            defaultVisible: false,
            currentVisible: false,
            vizKey: 'pncJunction',
        },
        lane: {
            defaultVisible: true,
            currentVisible: true,
            vizKey: 'lane',
        },
        road: {
            defaultVisible: false,
            currentVisible: false,
            vizKey: 'road',
        },
        signal: {
            defaultVisible: true,
            currentVisible: true,
            vizKey: 'signal',
        },
        stopSign: {
            defaultVisible: true,
            currentVisible: true,
            vizKey: 'stopSign',
        },
        yieldSign: {
            defaultVisible: true,
            currentVisible: true,
            vizKey: 'yieldSign',
        },
        speedBump: {
            defaultVisible: true,
            currentVisible: true,
            vizKey: 'speedBump',
        },
        parkingSpace: {
            defaultVisible: true,
            currentVisible: true,
            vizKey: 'parkingSpace',
        },
        barrierGate: {
            defaultVisible: true,
            currentVisible: true,
            vizKey: 'barrierGate',
        },
        area: {
            defaultVisible: true,
            currentVisible: true,
            vizKey: 'area',
        },
        parkingSpaceId: {
            defaultVisible: false,
            currentVisible: false,
            vizKey: 'parkingSpaceId',
        },
        laneId: {
            defaultVisible: false,
            currentVisible: false,
            vizKey: 'laneId',
        },
        egoBoudingBox: {
            defaultVisible: false,
            currentVisible: false,
            vizKey: 'boudingBox',
        },
    },
};

export const formatLayerParams = (params: { [key: string]: subMenuParams }) => {
    const result: { [key: string]: { [key: string]: boolean } } = {};
    Object.keys(params).forEach((key) => {
        const subLayerMenu = params[key];
        Object.keys(subLayerMenu).forEach((subKey) => {
            const subParams = subLayerMenu[subKey];
            result[key] = result[key] || {};
            result[key][subParams.vizKey] = subParams.currentVisible;
        });
    });
    return result;
};

export const localLayerMenuParamsManager = new LocalStorage(KEY_MANAGER.layerMenuParams);

export const getCurrentLayerParams = () => {
    const localStorageLayerMenuParams = localLayerMenuParamsManager.get();

    return localStorageLayerMenuParams || layerMenuParams;
};
