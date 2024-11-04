import { BehaviorSubject, filter, finalize, from, map, Observable, Subject, tap, timer } from 'rxjs';
import { debounceTime, throttle } from 'rxjs/operators';
import Logger from '@dreamview/log';
import { isEqual, isNil } from 'lodash';
import WorkerPoolManager from '@dreamview/dreamview-core/src/worker/WorkerPoolManager';
import { HandleMessageType } from '@dreamview/dreamview-core/src/services/models/response-message.model';
import { RequestDataType } from '@dreamview/dreamview-core/src/services/models/request-message.model';
import CountedSubject from '@dreamview/dreamview-core/src/util/CountedSubject';
import MultiKeyMap from '@dreamview/dreamview-core/src/util/MultiKeyMap';
import { Emptyable, Nullable } from '@dreamview/dreamview-core/src/util/similarFunctions';
import { genereateRequestId } from '@dreamview/dreamview-core/src/util/genereateRequestId';
import MessageQueue from '@dreamview/dreamview-core/src/util/MessageQueue';
import ChildWsWorkerClass from '@dreamview/dreamview-core/src/worker/child-ws.worker-class';
import { WorkerFactory } from '@dreamview/dreamview-core/src/worker/WorkerFactory';
import DecoderWorkerClass from '@dreamview/dreamview-core/src/worker/decoder.worker-class';
import { StreamDataNames } from '@dreamview/dreamview-core/src/services/api/types';
import { perfMonitor, DreamviewAnalysis } from '@dreamview/dreamview-analysis';
import { WebSocketConnection } from './websocket-connect.service';
import { config, requestIdleCallbackTimeout } from './constant';
import {
    ConnectionStatusEnum,
    isMessageType,
    MetaActionType,
    MetadataItem,
    RequestMessage,
    RequestMessageActionEnum,
    RequestStreamMessage,
    ResponseMessage,
    ResponseMessageActionEnum,
    SocketNameEnum,
    StreamMessage,
    StreamMessageData,
    SubscribePayload,
} from './type';
import { PluginManager } from './plugins/PluginManager';
import { WebSocketPlugin } from './plugins/type';
import MapMessageHandlerPlugin from './plugins/MapMessageHandlerPlugin';
import { indexedDBStorage } from '../../util/indexedDB/IndexedDBStorage';
import { ProtoLoader } from '../../util/ProtoLoader';
import { performanceController } from '../../util/PerformanceMonitor';
import { ConnectionManager } from './connection-manager';

const logger = Logger.getInstance('WebSocketManager');

const dataFrequencyMs = 100;

// 取消订阅延时，防抖
const unsubscribeDelay = 300;

interface IArrayItem {
    dataName: StreamDataNames;
    protoPath: string;
    channelName?: string;
}

export enum IEventName {
    ChannelTotal = 'channelTotal',
    ChannelChange = 'channelChange',
    BaseProtoChange = 'baseProtoChange',
}

export class WebSocketManager {
    connectionManager = new ConnectionManager();

    private childWsManagerQueue = new MessageQueue<string>({
        name: 'WebSocketManager',
    });

    readonly mainConnection: WebSocketConnection;

    private readonly pluginConnection: WebSocketConnection;

    private readonly protoLoader = new ProtoLoader();

    private registeInitEvent = new Map();

    private activeWorkers: { [key: string]: ChildWsWorkerClass } = {};

    // 子socket的数据流频率控制
    private throttleDuration = new BehaviorSubject<number>(100);

    // 当前数据帧率
    private frameRate = 10;

    private pluginManager = new PluginManager();

    private metadata: MetadataItem[] = [];

    private metadataSubject: BehaviorSubject<MetadataItem[]> = new BehaviorSubject<MetadataItem[]>([]);

    readonly initProtoFiles: string[] = [
        'modules/common_msgs/basic_msgs/error_code.proto',
        'modules/common_msgs/basic_msgs/header.proto',
        'modules/common_msgs/dreamview_msgs/hmi_status.proto',
        'modules/common_msgs/basic_msgs/geometry.proto',
        'modules/common_msgs/map_msgs/map_id.proto',
    ];

    private dataSubjects: MultiKeyMap<
        { name: string; channel?: Emptyable<string> },
        CountedSubject<StreamMessageData<unknown> | unknown>
    > = new MultiKeyMap();

    private responseResolvers: {
        [requestId: string]: {
            resolver: (response: any) => void;
            reject?: (error: any) => void;
            shouldDelete: boolean;
        };
    } = {};

    private workerPoolManager = new WorkerPoolManager<StreamMessage>({
        name: 'decoderWorkerPool',
        workerFactory: new WorkerFactory<StreamMessage>(() => new DecoderWorkerClass()),
    });

    constructor(mainUrl: string = config.mainUrl, pluginUrl: string = config.pluginUrl) {
        this.registerPlugin([new MapMessageHandlerPlugin()]);
        this.mainConnection = new WebSocketConnection(mainUrl);
        this.pluginConnection = new WebSocketConnection(pluginUrl);
        this.mainConnection.receivedMessages$.subscribe((msg) =>
            this.handleMessage(<HandleMessageType>msg, SocketNameEnum.MAIN),
        );
        this.pluginConnection.receivedMessages$.subscribe((msg) =>
            this.handleMessage(<HandleMessageType>msg, SocketNameEnum.PLUGIN),
        );

        this.connectionManager.set(SocketNameEnum.MAIN, this.mainConnection);
        this.connectionManager.set(SocketNameEnum.PLUGIN, this.pluginConnection);

        this.loadInitProtoFiles();

        this.metadataSubject.pipe(debounceTime(200)).subscribe(() => {
            this.consumeChildWsManagerQueue();
            const protoList: {
                level0: IArrayItem[];
                level1: IArrayItem[];
                level2: IArrayItem[];
            } = {
                level0: [],
                level1: [],
                level2: [],
            };

            const protoFiles: string[] = [];

            this.metadata.forEach((item) => {
                if (!item.differentForChannels) {
                    protoList.level0.push({ dataName: item.dataName, protoPath: item.protoPath });
                    protoFiles.push(`${item.protoPath}`);
                } else if (item.protoPath) {
                    protoList.level1.push({ dataName: item.dataName, protoPath: item.protoPath });
                    protoFiles.push(`${item.protoPath}`);
                } else {
                    item.channels.forEach((channel) => {
                        protoList.level2.push({
                            dataName: item.dataName,
                            protoPath: channel.protoPath,
                            channelName: channel.channelName,
                        });
                        protoFiles.push(`${item.protoPath}`);
                    });
                }
            });
            // console.log("protodata list: ", protoList);

            protoFiles.forEach((file) => {
                this.protoLoader.loadProto(file).catch((error) => {
                    logger.error(error);
                });
            });

            if (this.metadata.length > 0) {
                this.triggerEvent(
                    IEventName.ChannelTotal,
                    protoList.level0.length + protoList.level1.length + protoList.level2.length,
                );
                protoList.level0.forEach((item) => {
                    this.protoLoader
                        .loadAndCacheProto(item.protoPath, {
                            dataName: item.dataName,
                        })
                        .catch((error) => {
                            logger.error(error);
                        })
                        .finally(() => {
                            this.triggerEvent(IEventName.ChannelChange);
                        });
                });

                protoList.level1.forEach((item) => {
                    this.protoLoader
                        .loadAndCacheProto(item.protoPath, {
                            dataName: item.dataName,
                        })
                        .catch((error) => {
                            logger.error(error);
                        })
                        .finally(() => {
                            this.triggerEvent(IEventName.ChannelChange);
                        });
                });
                protoList.level2.forEach((item) => {
                    this.protoLoader
                        .loadAndCacheProto(item.protoPath, {
                            dataName: item.dataName,
                            channelName: item.channelName,
                        })
                        .catch((error) => {
                            logger.error(error);
                        })
                        .finally(() => {
                            this.triggerEvent(IEventName.ChannelChange);
                        });
                });
            }
        });
        if (process.env.NODE_ENV === 'development') {
            setInterval(() => {
                this.workerPoolManager.visualize();
            }, 1000);
        }

        performanceController.logicController$.subscribe((isHighLoad) => {
            logger.debug(`当前处于${isHighLoad ? '高负载' : '正常'}状态`);
            if (isHighLoad && this.frameRate > 5) {
                this.frameRate -= 1;
            } else if (!isHighLoad && this.frameRate < 10) {
                this.frameRate += 1;
            }
            DreamviewAnalysis.logData('wsFrameRate', this.frameRate, {
                useStatistics: {
                    useMax: true,
                    useMin: true,
                },
            });
            this.throttleDuration.next(Math.floor(1000 / this.frameRate));
        });
    }

    // 初始化其他socket请求用于请求响应
    public initBySocketName(socketName: SocketNameEnum) {
        if (!this.connectionManager.has(socketName)) {
            this.connectionManager
                .get(socketName)
                .receivedMessages$.subscribe((msg) => this.handleMessage(<HandleMessageType>msg, socketName));
        }
    }

    loadInitProtoFiles() {
        this.initProtoFiles.forEach((file) => {
            this.protoLoader
                .loadProto(file)
                .catch((error) => {
                    logger.error(error);
                })
                .finally(() => {
                    this.triggerEvent(IEventName.BaseProtoChange);
                });
        });
    }

    public registerPlugin(plugins: WebSocketPlugin[]): void {
        plugins.forEach((plugin) => this.pluginManager.registerPlugin(plugin));
    }

    private triggerEvent(eventName: IEventName, value?: any) {
        this.registeInitEvent.get(eventName)?.forEach((callback: any) => {
            callback(value);
        });
    }

    public addEventListener(eventName: IEventName, callback: any) {
        let events = this.registeInitEvent.get(eventName);
        if (!events) {
            this.registeInitEvent.set(eventName, []);
            events = this.registeInitEvent.get(eventName);
        }
        events.push(callback);
    }

    public removeEventListener(eventName: IEventName, callback: any) {
        const events = this.registeInitEvent.get(eventName);
        if (!events) {
            this.registeInitEvent.set(eventName, []);
        } else {
            this.registeInitEvent.set(
                eventName,
                events.filter((item: any) => item !== callback),
            );
        }
    }

    handleMessage(msg: HandleMessageType, socketName: SocketNameEnum) {
        logger.debug(`Received message from ${socketName}, message: ${JSON.stringify(msg, null, 0)}`);

        // 安全校验
        if (!msg?.action) {
            logger.error(`Received message from ${socketName}, but action is undefined`);
            return;
        }

        if (msg?.data?.info?.code === undefined) {
            logger.error(`Received message from ${socketName}, but code is undefined`);
            return;
        }

        if (msg?.data?.info?.code !== 0) {
            logger.error(
                `Received error message from ${socketName}, message: ${JSON.stringify(msg.data.info, null, 0)}`,
            );
        }

        if (msg.action === ResponseMessageActionEnum.METADATA_MESSAGE_TYPE) {
            const metadata = Object.values(msg.data.info.data.dataHandlerInfo);
            this.setMetadata(metadata);
            this.mainConnection.connectionStatus$.next(ConnectionStatusEnum.METADATA);
        } else if (msg.action === ResponseMessageActionEnum.METADATA_JOIN_TYPE) {
            const sourceDataForJoin = Object.values(msg.data.info.data.dataHandlerInfo);
            const metadata = this.updateMetadataChannels(this.metadata, 'join', sourceDataForJoin);
            this.setMetadata(metadata);
        } else if (msg.action === ResponseMessageActionEnum.METADATA_LEAVE_TYPE) {
            const sourceDataForLeave = Object.values(msg.data.info.data.dataHandlerInfo);
            const metadata = this.updateMetadataChannels(this.metadata, 'leave', sourceDataForLeave);
            this.setMetadata(metadata);
        } else if (msg.action === ResponseMessageActionEnum.RESPONSE_MESSAGE_TYPE) {
            if (msg && this.responseResolvers[msg.data.requestId]) {
                if (msg.data.info.code === 0) {
                    this.responseResolvers[msg.data.requestId].resolver(msg);
                } else {
                    this.responseResolvers[msg.data.requestId].reject(msg);
                }
                if (this.responseResolvers[msg.data.requestId].shouldDelete) {
                    delete this.responseResolvers[msg.data.requestId];
                }
            }
        }
    }

    /**
     * Updates or adds channels in a target MetadataItem array based on a specified action type
     * ('join' or 'leave') and source data. New MetadataItem objects are created
     * for new dataNames in case of 'join' action.
     * @param targetArray Target MetadataItem array to update
     * @param actionType Action type ('join' or 'leave')
     * @param sourceArray Source MetadataItem array
     */
    updateMetadataChannels(
        targetArray: MetadataItem[],
        actionType: MetaActionType,
        sourceArray: MetadataItem[],
    ): MetadataItem[] {
        const targetMap = new Map<string, MetadataItem>(targetArray.map((item) => [item.dataName, item]));

        sourceArray.forEach((sourceItem) => {
            const { dataName, channels } = sourceItem;
            let targetItem = targetMap.get(dataName);

            if (!targetItem) {
                targetItem = { dataName, channels: [] } as MetadataItem;
                targetMap.set(dataName, targetItem);
            } else {
                // 创建 targetItem 的浅副本
                targetItem = { ...targetItem };
            }

            if (actionType === 'join') {
                channels.forEach((newChannel) => {
                    if (!targetItem.channels.some((ch) => ch.channelName === newChannel.channelName)) {
                        targetItem.channels = [...targetItem.channels, newChannel];
                    }
                });
            } else if (actionType === 'leave') {
                targetItem.channels = targetItem.channels.filter(
                    (ch) => !channels.some((channelToRemove) => ch.channelName === channelToRemove.channelName),
                );
            }

            // 更新 targetMap 中的 targetItem
            targetMap.set(dataName, targetItem);
        });

        return Array.from(targetMap.values());
    }

    connectMain(retries = 3, retryInterval = 1000) {
        return this.mainConnection.connect(retries, retryInterval);
    }

    isMainConnected() {
        return this.mainConnection.isConnected();
    }

    connectPlugin(retries = 3, retryInterval = 1000) {
        return this.pluginConnection.connect(retries, retryInterval);
    }

    isPluginConnected() {
        return this.pluginConnection.isConnected();
    }

    disconnect() {
        logger.debug('Disconnected from all sockets');
        this.mainConnection.disconnect();
        this.pluginConnection.disconnect();
        Object.entries(this.activeWorkers).forEach(([name, childWsService]) => {
            childWsService.disconnect();
            from(this.dataSubjects.get({ name })).subscribe((subject) => {
                if (subject) {
                    subject.complete();
                }
            });
        });
    }

    public getMetadata(): MetadataItem[] {
        return this.metadata;
    }

    public setMetadata(metadata: MetadataItem[]): void {
        // meta只有更新后才会触发，防止重复播包时数据重置。
        if (!isEqual(this.metadata, metadata)) {
            this.metadata = metadata;
            this.metadataSubject.next(metadata);
            indexedDBStorage
                .getStoreManager('DreamviewPlus')
                .then(
                    (storeManager) => storeManager.setItem('metadata', metadata),
                    (error) => logger.error(error),
                )
                .then(() => logger.debug('metadata is saved to indexedDB'));
        } else {
            logger.debug('Metadata is not changed');
        }
    }

    get metadata$(): Observable<MetadataItem[]> {
        return this.metadataSubject.asObservable().pipe(debounceTime(100));
    }

    private connectChildSocket(name: string): void {
        const metadata = this.metadata.find((m) => m.dataName === name);

        if (!metadata) {
            logger.error(`Cannot find metadata for ${name}`);
            return;
        }

        if (!this.activeWorkers[name]) {
            this.activeWorkers[name] = new ChildWsWorkerClass(
                name as StreamDataNames,
                `${config.baseURL}/${metadata.websocketInfo.websocketName}`,
            ).connect();
        }

        this.activeWorkers[name].socketMessage$
            .pipe(throttle(() => timer(this.throttleDuration.value)))
            .subscribe((message) => {
                if (isMessageType(message, 'SOCKET_MESSAGE')) {
                    const {
                        data,
                        // performance,
                    } = message.payload as StreamMessage;
                    // 性能分析
                    // if (performance) {
                    //     const now = Date.now();
                    //     const start = performance.startTimestamp;
                    //     console.log(`数据${name}子通讯传输的数据：${now - start}ms`);
                    // }
                    this.workerPoolManager
                        .dispatchTask(
                            {
                                type: 'SOCKET_STREAM_MESSAGE',
                                payload: <StreamMessage>message.payload,
                                transferList: [data.buffer],
                            },
                            {
                                callback: () => {
                                    perfMonitor.mark(`dataDeserializeStart-${name}`);
                                },
                            },
                        )
                        .then(
                            (response) => {
                                // 性能测试：数据反序列化传输时长
                                // @ts-ignore
                                // if (response?.result?.performance) {
                                //     const now = Date.now();
                                //     const start = response?.result?.performance.startTimestamp;
                                //     console.log(
                                //         // @ts-ignore
                                //         `数据${response?.result?.dataName}${
                                //             // @ts-ignore
                                //             response?.result?.channelName
                                //         }反序列化传输返回的数据：${now - start}ms`,
                                //     );
                                // }
                                if (response.success) {
                                    perfMonitor.mark(`dataDeserializeEnd-${name}`);
                                    perfMonitor.measure(
                                        `dataDeserialize-${name}`,
                                        `dataDeserializeStart-${name}`,
                                        `dataDeserializeEnd-${name}`,
                                    );
                                    this.dataSubjects.getByExactKey({ name })?.next(response.result);
                                }
                            },
                            (error) => {
                                logger.error(error);
                            },
                        );
                }
            });
    }

    public sendSubscriptionMessage<Param>(
        action: RequestMessageActionEnum,
        name: string,
        channel: Nullable<string>,
        option?: {
            param?: Param;
            dataFrequencyMs?: number;
        },
    ): void {
        if (!this.mainConnection.isConnected()) {
            logger.error('Main socket is not connected');
            return;
        }

        const metadata = this.metadata.find((m) => m.dataName === name);

        if (!metadata) {
            logger.error(`Cannot find metadata for ${name}`);
            return;
        }

        const info: SubscribePayload = {
            websocketName: metadata.websocketInfo.websocketName,
            ...(isNil(channel) ? {} : { channelName: channel }),
            ...(isNil(option?.param) ? {} : { param: option.param }),
            dataFrequencyMs: option?.dataFrequencyMs ?? dataFrequencyMs,
        };

        this.mainConnection.sendMessage({
            action,
            type: action,
            data: {
                name: action,
                source: 'dreamview',
                info,
                sourceType: 'websocktSubscribe',
                targetType: 'module',
                requestId: action,
            },
        } as RequestStreamMessage);
    }

    /**
     * 初始化子socket
     * @param name websocketName
     * @private
     */
    private initChildSocket(name: string): void {
        if (this.activeWorkers[name] === undefined) {
            this.childWsManagerQueue.enqueue(name);
        }
        this.consumeChildWsManagerQueue();
    }

    private consumeChildWsManagerQueue() {
        // 用requestIdleCallback排队执行的函数
        const idleConsume = () => {
            let maxIterations = this.childWsManagerQueue.size;

            // 仅当浏览器空闲时，才开始处理队列中的消息
            while (!this.childWsManagerQueue.isEmpty() && maxIterations > 0) {
                const name = this.childWsManagerQueue.dequeue();
                // 检查 metadata 是否存在
                const metadata = this.metadata.find((m) => m.dataName === name);

                // 如果 metadata 存在且 worker 未激活，则进行连接
                if (metadata && this.activeWorkers[name] === undefined) {
                    logger.debug(`Connecting to ${name}`);
                    this.connectChildSocket(name);
                } else {
                    // metadata中没有相关子消息，重新放回队列
                    this.childWsManagerQueue.enqueue(name);
                }
                maxIterations -= 1;
            }
        };

        requestIdleCallback(idleConsume, { timeout: requestIdleCallbackTimeout });
    }

    /**
     * 订阅指定name的数据
     * @param name 数据名
     * @param option 参数
     */
    public subscribeToData<T, Param>(
        name: string,
        option?: {
            param?: Param;
            dataFrequencyMs?: number;
        },
    ): CountedSubject<T> {
        this.initChildSocket(name);
        if (this.dataSubjects.getByExactKey({ name }) === undefined) {
            this.dataSubjects.set({ name }, new CountedSubject(name));
            this.sendSubscriptionMessage(RequestMessageActionEnum.SUBSCRIBE_MESSAGE_TYPE, name, null, option);
        }

        const subject$ = this.dataSubjects.getByExactKey({ name }) as CountedSubject<StreamMessageData<unknown>>;

        const plugins = this.pluginManager.getPluginsForDataName(name);

        const inflowPlugins = this.pluginManager.getPluginsForInflowDataName(name);

        return <CountedSubject<T>>subject$.pipe(
            tap((data) => {
                inflowPlugins.forEach((plugin) => plugin.handleInflow?.(data?.data, this.dataSubjects, this));
            }),
            // @ts-ignore
            map((data) => plugins.reduce((acc, plugin) => plugin.handleSubscribeData(acc), data?.data as T)),
            finalize(() => {
                const subscribersCount = subject$.count;
                const isCompleted = subject$.completed;
                if (isCompleted) return;
                if (subscribersCount === 0) {
                    setTimeout(() => {
                        const delaySubscribersCount = subject$.count;
                        if (delaySubscribersCount === 0) {
                            this.sendSubscriptionMessage(
                                RequestMessageActionEnum.UNSUBSCRIBE_MESSAGE_TYPE,
                                name,
                                null,
                                option,
                            );
                            this.dataSubjects.delete({ name }, (countedSubject) => countedSubject.complete());
                        }
                    }, unsubscribeDelay);
                }
            }),
        );
    }

    /**
     * 订阅指定name、channel的数据
     * @param name 数据名
     * @param channel 频道名
     * @param option 参数
     */
    public subscribeToDataWithChannel<T, Param>(
        name: string,
        channel: string,
        option?: {
            param?: Param;
            dataFrequencyMs?: number;
        },
    ): CountedSubject<T> {
        this.initChildSocket(name);

        if (this.dataSubjects.getByExactKey({ name }) === undefined) {
            // 初始化上游数据源
            this.dataSubjects.set({ name }, new CountedSubject(name));
        }

        if (this.dataSubjects.getByExactKey({ name, channel }) === undefined) {
            this.sendSubscriptionMessage(RequestMessageActionEnum.SUBSCRIBE_MESSAGE_TYPE, name, channel, option);
            this.dataSubjects.set({ name, channel }, new CountedSubject(name, channel));
        }

        const upstream = this.dataSubjects.getByExactKey({ name }) as CountedSubject<StreamMessageData<unknown>>;
        const downstream = this.dataSubjects.getByExactKey({ name, channel }) as CountedSubject<T>;

        upstream
            .pipe(filter((data) => data?.channelName === channel))
            .subscribe((filterData) => downstream.next(<T>filterData.data));

        return <CountedSubject<T>>downstream.pipe(
            finalize(() => {
                const subscribersCount = downstream.count;
                const isCompleted = downstream.completed;
                if (isCompleted) return;
                if (subscribersCount === 0) {
                    setTimeout(() => {
                        const delaySubscribersCount = downstream.count;
                        if (delaySubscribersCount === 0) {
                            this.sendSubscriptionMessage(
                                RequestMessageActionEnum.UNSUBSCRIBE_MESSAGE_TYPE,
                                name,
                                channel,
                                option,
                            );

                            this.dataSubjects.deleteByExactKey({ name, channel }, (countedSubject) =>
                                countedSubject.complete(),
                            );
                        }
                    }, unsubscribeDelay);
                }

                // 如果没有下游订阅者，就取消上游数据源的订阅
                if (this.dataSubjects.countIf((subject) => subject.name === name) === 1) {
                    // fixme: 暂时不删除这个主题
                    // this.dataSubjects.delete({ name }, (countedSubject) => countedSubject.complete());
                    // this.activeSockets[name]?.complete();
                    // delete this.activeSockets[name];
                }
            }),
        );
    }

    /**
     * 订阅指定name，模糊匹配channel的数据
     * @param name 数据名
     */
    public subscribeToDataWithChannelFuzzy<T>(name: string): Emptyable<CountedSubject<T>> {
        const theNameDataSubjects = this.dataSubjects.get<CountedSubject<T>>({ name });
        return theNameDataSubjects?.filter((subject) => subject.channel !== undefined)[0];
    }

    public request<Req, Res>(
        requestData: RequestDataType<Req>,
        socket: SocketNameEnum = SocketNameEnum.MAIN,
        requestId: string | 'noResponse' = genereateRequestId(requestData.type),
    ): Promise<ResponseMessage<Res>> {
        if (requestId === 'noResponse') {
            this.sendMessage<Req>(
                {
                    ...requestData,
                    data: {
                        ...requestData.data,
                        requestId,
                    },
                    action: RequestMessageActionEnum.REQUEST_MESSAGE_TYPE,
                },
                socket,
            );
            return Promise.resolve(null);
        }

        return new Promise((resolve, reject) => {
            this.responseResolvers[requestId] = {
                resolver: resolve,
                reject,
                shouldDelete: true,
            };
            this.sendMessage<Req>(
                {
                    ...requestData,
                    data: {
                        ...requestData.data,
                        requestId,
                    },
                    action: RequestMessageActionEnum.REQUEST_MESSAGE_TYPE,
                },
                socket,
            );
        });
    }

    public requestStream<Req, Res>(
        requestData: RequestDataType<Req>,
        socket: SocketNameEnum = SocketNameEnum.MAIN,
        requestId: string = genereateRequestId(requestData.type),
    ): Observable<ResponseMessage<Res>> {
        const responseSubject = new Subject<ResponseMessage<Res>>();

        this.responseResolvers[requestId] = {
            resolver: (response: ResponseMessage<Res>) => {
                responseSubject.next(response);
            },
            reject: (response: ResponseMessage<Res>) => {
                responseSubject.error(response);
            },
            shouldDelete: false,
        };
        this.sendMessage(
            {
                ...requestData,
                data: {
                    ...requestData.data,
                    requestId,
                },
                action: RequestMessageActionEnum.REQUEST_MESSAGE_TYPE,
            },
            socket,
        );

        return responseSubject.asObservable().pipe(
            finalize(() => {
                delete this.responseResolvers[requestId];
            }),
        );
    }

    public sendMessage<T>(request: RequestMessage<T>, socket: SocketNameEnum = SocketNameEnum.MAIN): void {
        const connection = this.connectionManager.get(socket);

        connection.sendMessage({ ...request });
    }
}

export const webSocketManager = new WebSocketManager();
