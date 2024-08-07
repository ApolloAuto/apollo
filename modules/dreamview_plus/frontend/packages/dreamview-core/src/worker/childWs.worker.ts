/* eslint-disable no-restricted-globals */
import { apollo } from '@dreamview/dreamview';
import Logger from '@dreamview/log';
import { delay, retryWhen, Subscription, take } from 'rxjs';
import { webSocket, WebSocketSubject } from 'rxjs/webSocket';
import { StreamDataNames } from '@dreamview/dreamview-core/src/services/api/types';
import { Nullable } from '@dreamview/dreamview-core/src/util/similarFunctions';
import {
    StreamMessage,
    WorkerMessage,
    WorkerMessageType,
} from '@dreamview/dreamview-core/src/services/WebSocketManager/type';
import { isNil } from 'lodash';

const decodeStreamData = apollo.dreamview.StreamData.decode;

const logger = Logger.getInstance('ChildWebSocketWorker');

let childSocket: WebSocketSubject<ArrayBuffer>;
let childSpt: Subscription;
const MAX_RETRIES = 5;

const deserializer = (data: unknown, name: string): Nullable<StreamMessage> => {
    try {
        if (data instanceof ArrayBuffer) {
            const arrayBuffer = new Uint8Array(data);
            return <StreamMessage>decodeStreamData(arrayBuffer);
        }

        if (typeof data === 'string') {
            return <StreamMessage>JSON.parse(data);
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
                self.postMessage({
                    type: 'SOCKET_OPEN',
                    payload: {
                        performance: {
                            startTimestamp: performance.now(),
                        },
                    },
                } as WorkerMessage<WorkerMessageType>);
            },
        },
        closeObserver: {
            next: (closeEvent) => {
                logger.debug('child websocket disconnected');
                self.postMessage({
                    type: 'SOCKET_CLOSED',
                    payload: {
                        performance: {
                            startTimestamp: performance.now(),
                        },
                    },
                } as WorkerMessage<WorkerMessageType>);
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
        .subscribe((buffer) => {
            const decodedData = deserializer(buffer, name);

            const { action, type, dataName, channelName, data } = decodedData;

            if (decodedData) {
                self.postMessage(
                    {
                        type: 'SOCKET_MESSAGE',
                        payload: {
                            action,
                            type,
                            dataName,
                            ...(isNil(channelName) ? {} : { channelName }),
                            data,
                            // performance: {
                            //     startTimestamp: Date.now(),
                            // },
                        },
                    } as WorkerMessage<WorkerMessageType>,
                    // @ts-ignore
                    [data.buffer],
                );
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
