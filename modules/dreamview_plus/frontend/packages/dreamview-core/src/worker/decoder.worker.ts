/* eslint-disable no-restricted-globals */
import Logger from '@dreamview/log';
import { Subject } from 'rxjs';
import {
    MetadataItem,
    StreamMessage,
    WorkerMessage,
    WorkerMessageType,
} from '@dreamview/dreamview-core/src/services/WebSocketManager/type';
import { indexedDBStorage } from '@dreamview/dreamview-core/src/util/indexedDB/IndexedDBStorage';
import { ProtoLoader } from '@dreamview/dreamview-core/src/util/ProtoLoader';
import { TaskInternal } from './type';

const logger = Logger.getInstance('decoderWorker');

const protoLoader = new ProtoLoader();

const subject = new Subject<TaskInternal<StreamMessage>>();

function loadProtoAsPromise(data: any, protoPath: string, msgType: string, config?: any) {
    return new Promise((resolve, reject) => {
        protoLoader.loadAndCacheProto(protoPath, config).then((root) => {
            try {
                const message = root.lookupType(msgType);
                const decodedData = message.decode(data);
                const objectData = message.toObject(decodedData, {
                    enums: String,
                    longs: String,
                });
                resolve(objectData);
            } catch (error) {
                reject(error);
            }
        });
    });
}

const release = () =>
    self.postMessage({
        success: false,
        result: null,
    });

subject.subscribe(
    async (message) => {
        try {
            const storeManager = await indexedDBStorage.getStoreManager<MetadataItem[]>('DreamviewPlus');

            const metadata: MetadataItem[] = (await storeManager.getItem('metadata')) || <MetadataItem[]>[];

            const { id, payload } = message;

            const { dataName, channelName, data } = payload || {};

            const dataNameInMetadata = metadata.some((item) => item.dataName === dataName);

            if (!dataNameInMetadata) {
                logger.error(`Data name ${dataName} not found in metadata`);
                release();
                throw new Error(`Data name ${dataName} not found in metadata`);
            }

            const dataNameMeta = metadata.find((item) => item.dataName === dataName);

            if (dataNameMeta.differentForChannels && !channelName) {
                logger.error('Channel name not found in message payload');
                release();
                throw new Error('Channel name not found in message payload');
            }

            const protoPath =
                dataNameMeta.protoPath ||
                dataNameMeta.channels.find((item) => item.channelName === channelName)?.protoPath;
            const msgType =
                dataNameMeta.msgType || dataNameMeta.channels.find((item) => item.channelName === channelName)?.msgType;

            const objectData = await loadProtoAsPromise(data, protoPath, msgType, {
                dataName,
                channelName,
            });

            self.postMessage({
                id,
                success: true,
                result: { ...payload, data: objectData },
            });
        } catch (e) {
            const { id } = message;
            self.postMessage({
                id,
                success: false,
                result: null,
            });
            throw new Error(e as string);
        }
    },
    (error) => {
        logger.error(error);
        release();
        throw new Error(error);
    },
);

function isMessageType<T extends WorkerMessageType>(
    message: WorkerMessage<WorkerMessageType>,
    type: T,
): message is WorkerMessage<T> {
    return message.type === type;
}

self.onmessage = (event: MessageEvent<WorkerMessage<WorkerMessageType>>) => {
    const message = event.data;
    try {
        if (isMessageType(message, 'SOCKET_STREAM_MESSAGE')) {
            subject.next(message);
        }
    } catch (e) {
        const { id } = message;
        self.postMessage({
            id,
            success: false,
            result: null,
        });
    }
};
