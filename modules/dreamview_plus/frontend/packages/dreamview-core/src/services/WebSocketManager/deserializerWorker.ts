/* eslint-disable no-restricted-globals */
import { apollo } from '@dreamview/dreamview';
import Logger from '@dreamview/log';
import { noop } from 'rxjs';
import { StreamMessage, StreamMessageData } from './type';
import { StreamDataNames } from '../api/types';
import { Nullable } from '../../util/similarFunctions';

const logger = Logger.getInstance('WebSocketManager');
const decodeStreamData = apollo.dreamview.StreamData.decode;
const toObjectStreamData = apollo.dreamview.StreamData.toObject;

const toObjectOptions = {
    enums: String,
    longs: String,
};

const getDeserializerMethods = (dataName: StreamMessage['dataName']) => {
    switch (dataName) {
        case StreamDataNames.SIM_WORLD:
            return {
                decode: apollo.dreamview.SimulationWorld.decode,
                toObject: apollo.dreamview.SimulationWorld.toObject,
            };
        case StreamDataNames.CAMERA:
            return {
                decode: apollo.dreamview.CameraUpdate.decode,
                toObject: apollo.dreamview.CameraUpdate.toObject,
            };
        case StreamDataNames.HMI_STATUS:
            return {
                decode: apollo.dreamview.HMIStatus.decode,
                toObject: apollo.dreamview.HMIStatus.toObject,
            };
        case StreamDataNames.POINT_CLOUD:
            return {
                decode: apollo.dreamview.PointCloud.decode,
                toObject: apollo.dreamview.PointCloud.toObject,
            };
        case StreamDataNames.Map:
            return {
                decode: apollo.hdmap.Map.decode,
                toObject: apollo.hdmap.Map.toObject,
            };
        case StreamDataNames.Obstacle:
            return {
                decode: apollo.dreamview.Obstacles.decode,
                toObject: apollo.dreamview.Obstacles.toObject,
            };
        default:
            return {
                decode: noop,
                toObject: noop,
            };
    }
};

const deserializerMethodsCache: { [dataName: string]: any } = {};

function getDeserializerMethodsCached(dataName: string) {
    if (!deserializerMethodsCache[dataName]) {
        deserializerMethodsCache[dataName] = getDeserializerMethods(dataName);
    }
    return deserializerMethodsCache[dataName];
}

const deserializer = (data: unknown, name: string): Nullable<StreamMessageData<unknown>> => {
    try {
        if (typeof data === 'string') {
            return JSON.parse(data);
        }
        if (data instanceof ArrayBuffer) {
            const arrayBuffer = new Uint8Array(data);
            const message = decodeStreamData(arrayBuffer);
            <StreamMessage>toObjectStreamData(message, toObjectOptions);
            const dataName = message?.dataName;
            if (dataName) {
                const { decode, toObject } = getDeserializerMethodsCached(dataName);
                const dataMessage = decode(message.data);

                return <StreamMessageData<unknown>>Object.assign(message, {
                    data: toObject(dataMessage, toObjectOptions),
                });
            }
        }
        return null;
    } catch (error) {
        logger.error(`Failed to decode message from ${name}, error: ${error}`);
        return null;
    }
};

self.onmessage = (event: MessageEvent<{ data: unknown; name: string }>) => {
    const { data, name } = event.data;

    const decodedData = deserializer(data, name);
    self.postMessage(decodedData);
};
