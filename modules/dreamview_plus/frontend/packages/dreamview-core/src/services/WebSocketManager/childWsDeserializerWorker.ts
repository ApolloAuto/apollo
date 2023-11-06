/* eslint-disable no-restricted-globals */
import memoize from 'lodash/memoize';
import { apollo } from '@dreamview/dreamview';
import Logger from '@dreamview/log';
import { delay, noop, retryWhen, Subscription, take } from 'rxjs';
import { webSocket, WebSocketSubject } from 'rxjs/webSocket';
import { StreamMessage, StreamMessageData, WorkerMessage, WorkerMessageType } from './type';
import { StreamDataNames } from '../api/types';
import { Nullable } from '../../util/similarFunctions';

const decodeStreamData = apollo.dreamview.StreamData.decode;
const toObjectStreamData = apollo.dreamview.StreamData.toObject;

const logger = Logger.getInstance('ChildWebSocketWorker');

const toObjectOptions = {
    enums: String,
    longs: String,
};

let childSocket: WebSocketSubject<ArrayBuffer>;
let childSpt: Subscription;
const MAX_RETRIES = 5;

const deserializationMethodsMapping: {
    [key in StreamDataNames]: {
        decode: (bytes: Uint8Array) => any;
        toObject: (message: any, options?: any) => any;
    };
} = {
    [StreamDataNames.SIM_WORLD]: {
        decode: apollo.dreamview.SimulationWorld.decode,
        toObject: apollo.dreamview.SimulationWorld.toObject,
    },
    [StreamDataNames.CAMERA]: {
        decode: apollo.dreamview.CameraUpdate.decode,
        toObject: apollo.dreamview.CameraUpdate.toObject,
    },
    [StreamDataNames.HMI_STATUS]: {
        decode: apollo.dreamview.HMIStatus.decode,
        toObject: apollo.dreamview.HMIStatus.toObject,
    },
    [StreamDataNames.POINT_CLOUD]: {
        decode: apollo.dreamview.PointCloud.decode,
        toObject: apollo.dreamview.PointCloud.toObject,
    },
    [StreamDataNames.Map]: {
        decode: apollo.hdmap.Map.decode,
        toObject: apollo.hdmap.Map.toObject,
    },
    [StreamDataNames.Obstacle]: {
        decode: apollo.dreamview.Obstacles.decode,
        toObject: apollo.dreamview.Obstacles.toObject,
    },
};

const getDeserializerMethods = memoize(
    (dataName: StreamMessage['dataName']) =>
        deserializationMethodsMapping[dataName] || { decode: noop, toObject: noop },
);

const deserializer = (data: unknown, name: string): Nullable<StreamMessageData<unknown>> => {
    try {
        if (typeof data === 'string') {
            return JSON.parse(data);
        }
        if (data instanceof ArrayBuffer) {
            const arrayBuffer = new Uint8Array(data);
            const message = decodeStreamData(arrayBuffer);
            <StreamMessage>toObjectStreamData(message, toObjectOptions);
            const dataName = message?.dataName as StreamDataNames;
            if (dataName) {
                const { decode, toObject } = getDeserializerMethods(dataName);
                const dataMessage = decode(message.data);

                return <StreamMessageData<unknown>>Object.assign(message, {
                    data: toObject(dataMessage, toObjectOptions),
                    performance: {
                        startTimestamp: performance.now(),
                    },
                });
            }
        }
        logger.error(`Failed to decode message from ${name}, data: ${data}`);
        return null;
    } catch (error) {
        logger.error(`Failed to decode message from ${name}, error: ${error}`);
        return null;
    }
};

const connectChildSocket = (url: string, name: StreamDataNames): void => {
    childSocket = webSocket({
        url,
        binaryType: 'arraybuffer',
        deserializer: (e) => e.data,
        openObserver: {
            next: () => {
                logger.debug('child websocket connected');
            },
        },
        closeObserver: {
            next: (closeEvent) => {
                logger.debug('child websocket disconnected');
                if (closeEvent.wasClean) {
                    logger.debug('child websocket closed gracefully');
                } else {
                    logger.debug(
                        `child websocket closed unexpectedly, error code: ${closeEvent.code}, error reason: ${closeEvent.reason}`,
                    );
                }
            },
        },
    });
    childSpt = childSocket
        .pipe(retryWhen((errors) => errors.pipe(take(MAX_RETRIES), delay(1000))))
        .subscribe((data) => {
            const decodedData = deserializer(data, name);

            if (decodedData) {
                self.postMessage({
                    type: 'SOCKET_MESSAGE',
                    payload: decodedData,
                } as WorkerMessage<WorkerMessageType>);
            }
        });
};

function isMessageType<T extends WorkerMessageType>(
    message: WorkerMessage<WorkerMessageType>,
    type: T,
): message is WorkerMessage<T> {
    return message.type === type;
}

self.onmessage = (event: MessageEvent<WorkerMessage<WorkerMessageType>>) => {
    const message = event.data;
    if (isMessageType(message, 'INIT_SOCKET') || isMessageType(message, 'RECONNECT_SOCKET')) {
        const { name, url } = message.payload;

        if (!childSocket?.closed) {
            connectChildSocket(url, name);
        }
    }

    if (isMessageType(message, 'DESTROY_SOCKET')) {
        childSpt?.unsubscribe();
        childSocket?.unsubscribe();
    }

    if (isMessageType(message, 'INTERRUPT_SOCKET')) {
        if (childSocket && !childSocket.closed) {
            childSocket.unsubscribe();
        }
    }
};
