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
            defaultVisible: true,
            currentVisible: true,
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
            defaultVisible: true,
            currentVisible: true,
            vizKey: 'laneMarker',
        },
        lidarSensor: {
            defaultVisible: true,
            currentVisible: true,
            vizKey: 'lidarSensor',
        },
        radarSensor: {
            defaultVisible: true,
            currentVisible: true,
            vizKey: 'radarSensor',
        },
        cameraSensor: {
            defaultVisible: true,
            currentVisible: true,
            vizKey: 'cameraSensor',
        },
        v2x: {
            defaultVisible: true,
            currentVisible: true,
            vizKey: 'v2x',
        },
    },
    Prediction: {
        priority: {
            defaultVisible: false,
            currentVisible: false,
            vizKey: 'obstaclePriority',
        },
        majorPredictionLine: {
            defaultVisible: false,
            currentVisible: false,
            vizKey: 'majorPredictionLine',
        },
        gaussianInfo: {
            defaultVisible: false,
            currentVisible: false,
            vizKey: 'gaussianInfo',
        },
        minorPredictionLine: {
            defaultVisible: false,
            currentVisible: false,
            vizKey: 'minorPredictionLine',
        },
        interactiveTag: {
            defaultVisible: false,
            currentVisible: false,
            vizKey: 'obstacleInteractiveTag',
        },
    },
    Routing: {
        routingLine: {
            defaultVisible: false,
            currentVisible: false,
            vizKey: 'routingLine',
        },
    },
    Decision: {
        mainDecision: {
            defaultVisible: false,
            currentVisible: false,
            vizKey: 'mainDecision',
        },
        obstacleDecision: {
            defaultVisible: false,
            currentVisible: false,
            vizKey: 'obstacleDecision',
        },
    },
    Planning: {
        planningCar: {
            defaultVisible: false,
            currentVisible: false,
            vizKey: 'planningCar',
        },
        planningTrajectory: {
            defaultVisible: false,
            currentVisible: false,
            vizKey: 'planningTrajectory',
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
            defaultVisible: false,
            currentVisible: false,
            vizKey: 'crosswalk',
        },
        clearArea: {
            defaultVisible: false,
            currentVisible: false,
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
            defaultVisible: false,
            currentVisible: false,
            vizKey: 'lane',
        },
        road: {
            defaultVisible: false,
            currentVisible: false,
            vizKey: 'road',
        },
        signal: {
            defaultVisible: false,
            currentVisible: false,
            vizKey: 'signal',
        },
        stopSign: {
            defaultVisible: false,
            currentVisible: false,
            vizKey: 'stopSign',
        },
        yieldSign: {
            defaultVisible: false,
            currentVisible: false,
            vizKey: 'yieldSign',
        },
        speedBump: {
            defaultVisible: false,
            currentVisible: false,
            vizKey: 'speedBump',
        },
        parkingSpace: {
            defaultVisible: false,
            currentVisible: false,
            vizKey: 'parkingSpace',
        },
        barrierGate: {
            defaultVisible: false,
            currentVisible: false,
            vizKey: 'barrierGate',
        },
        area: {
            defaultVisible: false,
            currentVisible: false,
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

export const localPointCloudManager = new LocalStorage(KEY_MANAGER.PointCloudLayerMenu);

export const getCurrentLayerParams = () => {
    const localStorageLayerMenuParams = localPointCloudManager.get();

    return localStorageLayerMenuParams || layerMenuParams;
};
