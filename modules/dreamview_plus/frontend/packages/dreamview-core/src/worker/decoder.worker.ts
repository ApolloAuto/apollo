/* eslint-disable no-restricted-globals */
import Logger from '@dreamview/log';
import { Subject, switchMap } from 'rxjs';
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

// 需要枚举转换的数据类型
const ENUM_DATA_TYPES = [
    'apollo.dreamview.CameraUpdate',
    'apollo.dreamview.HMIStatus',
    'apollo.dreamview.SimulationWorld',
    'apollo.dreamview.Obstacles',
    'apollo.hdmap.Map',
];

// 数据判断函数，需要包含缓存能力，不要每次遍历
const isEnumDataType = (() => {
    const cache = new Map<string, boolean>();
    return (msgType: string) => {
        if (cache.has(msgType)) {
            return cache.get(msgType);
        }
        const result = ENUM_DATA_TYPES.includes(msgType);
        cache.set(msgType, result);
        return result;
    };
})();

async function loadProtoAsPromise(data: any, protoPath: string, msgType: string, config?: any) {
    try {
        const root = await protoLoader.loadAndCacheProto(protoPath, config);

        const message = root.lookupType(msgType);

        let decodedData: any = message.decode(data);
        if (isEnumDataType(msgType)) {
            decodedData = message.toObject(decodedData, {
                enums: String,
            });
        }
        return decodedData;
    } catch (e) {
        console.error(e);
        return Promise.reject(e);
    }
}

const release = (id?: string) =>
    self.postMessage({
        id,
        success: false,
        result: null,
    });

let storeManager: any;

subject
    .pipe(
        switchMap(async (message) => {
            if (!storeManager) {
                storeManager = await indexedDBStorage.getStoreManager('DreamviewPlus');
            }
            return message; // Pass the message along with the initialized storeManager
        }),
    )
    .subscribe(async (message) => {
        try {
            if (!storeManager) release();
            const metadata: MetadataItem[] = (await storeManager?.getItem('metadata')) || <MetadataItem[]>[];

            if (metadata.length === 0) release();

            const { id, payload } = message;

            const {
                dataName,
                channelName,
                data,
                // performance,
            } = payload || {};

            // if (performance) {
            //     const now = Date.now();
            //     const start = performance.startTimestamp;
            //     console.log(`数据${dataName}${channelName}进入反序列化传输的数据：${now - start}ms`);
            // }

            const dataNameMeta = metadata.find((item) => item.dataName === dataName);

            if (!dataNameMeta) {
                logger.error(`Data name ${dataName} not found in metadata`);
                throw new Error(`Data name ${dataName} not found in metadata`);
            }

            if (dataNameMeta.differentForChannels && !channelName) {
                logger.error('Channel name not found in message payload');
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
            }).catch(() => {
                release(id);
                throw new Error(`Failed to decode data for ${dataName} ${channelName}`);
            });

            self.postMessage({
                id,
                success: true,
                result: {
                    ...payload,
                    data: objectData,
                    // performance: { startTimestamp: Date.now() }，
                },
            });
        } catch (e) {
            const { id } = message;
            release(id);
            throw new Error(e as string);
        }
    });

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
            // @ts-ignore
            subject.next(message);
        }
    } catch (e) {
        // @ts-ignore
        const { id } = message;
        self.postMessage({
            id,
            success: false,
            result: null,
        });
    }
};
