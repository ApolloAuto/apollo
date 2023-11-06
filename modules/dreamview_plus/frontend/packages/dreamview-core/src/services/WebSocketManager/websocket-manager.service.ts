import { BehaviorSubject, filter, finalize, from, map, Observable, Subject, tap } from 'rxjs';
import Logger from '@dreamview/log';
import { isEqual, isNil } from 'lodash';
import { HandleMessageType } from '@dreamview/dreamview-core/src/services/models/response-message.model';
import { RequestDataType } from '@dreamview/dreamview-core/src/services/models/request-message.model';
import CountedSubject from '@dreamview/dreamview-core/src/util/CountedSubject';
import MultiKeyMap from '@dreamview/dreamview-core/src/util/MultiKeyMap';
import { Emptyable, Nullable } from '@dreamview/dreamview-core/src/util/similarFunctions';
import { genereateRequestId } from '@dreamview/dreamview-core/src/util/genereateRequestId';
import { WebSocketConnection } from './websocket-connect.service';
import { config } from './constant';
import {
    ConnectionStatusEnum,
    isMessageType,
    MetadataItem,
    RequestMessage,
    RequestMessageActionEnum,
    RequestStreamMessage,
    ResponseMessage,
    ResponseMessageActionEnum,
    SocketNameEnum,
    StreamMessageData,
    SubscribePayload,
} from './type';
import { PluginManager } from './plugins/PluginManager';
import { WebSocketPlugin } from './plugins/type';
import MapMessageHandlerPlugin from './plugins/MapMessageHandlerPlugin';
import ChildWsService from './child-ws.service';
import { StreamDataNames } from '../api/types';

const logger = Logger.getInstance('WebSocketManager');

const dataFrequencyMs = 100;

export class WebSocketManager {
    private readonly mainConnection: WebSocketConnection;

    private readonly pluginConnection: WebSocketConnection;

    private activeWorkers: { [key: string]: ChildWsService } = {};

    private pluginManager = new PluginManager();

    private metadata: MetadataItem[] = [];

    private metadataSubject: BehaviorSubject<MetadataItem[]> = new BehaviorSubject<MetadataItem[]>([]);

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
    }

    public registerPlugin(plugins: WebSocketPlugin[]): void {
        plugins.forEach((plugin) => this.pluginManager.registerPlugin(plugin));
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
        } else if (msg.action === ResponseMessageActionEnum.RESPONSE_MESSAGE_TYPE) {
            if (msg && this.responseResolvers[msg.data.requestId]) {
                if (msg.data.info.code === 0) {
                    this.responseResolvers[msg.data.requestId].resolver(msg);
                }
                if (this.responseResolvers[msg.data.requestId].shouldDelete) {
                    delete this.responseResolvers[msg.data.requestId];
                }
            }
        }
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
        } else {
            logger.debug('Metadata is not changed');
        }
    }

    get metadata$(): Observable<MetadataItem[]> {
        return this.metadataSubject.asObservable();
    }

    private connectChildSocket(name: string): void {
        const metadata = this.metadata.find((m) => m.dataName === name);
        if (!metadata) return;

        if (!this.activeWorkers[name]) {
            this.activeWorkers[name] = new ChildWsService(
                name as StreamDataNames,
                `${config.baseURL}/${metadata.websocketInfo.websocketName}`,
            ).connect();
        }

        this.activeWorkers[name].socketMessage$.subscribe((message) => {
            if (isMessageType(message, 'SOCKET_MESSAGE')) {
                const endTimestamp = performance.now();
                const startTimestamp = message.payload?.performance?.startTimestamp;
                if (startTimestamp) {
                    logger.debug(`Message from ${name} took ${endTimestamp - startTimestamp}ms to transfer`);
                }
                this.dataSubjects.getByExactKey({ name })?.next(message.payload);
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

        const info: SubscribePayload = {
            websocketName: name,
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
            this.connectChildSocket(name);
        }
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
                    this.sendSubscriptionMessage(RequestMessageActionEnum.UNSUBSCRIBE_MESSAGE_TYPE, name, null, option);

                    this.dataSubjects.delete({ name }, (countedSubject) => countedSubject.complete());
                    // 不再主动关闭socket，可以复用。
                    // this.activeSockets[name].complete();
                    // delete this.activeSockets[name];
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
            // this.mainConnection.sendMessage({
            //     action: RequestMessageActionEnum.SUBSCRIBE_MESSAGE_TYPE,
            //     type: RequestMessageActionEnum.SUBSCRIBE_MESSAGE_TYPE,
            //     data: {
            //         name: 'subscribe',
            //         source: 'dreamview',
            //         info: {
            //             websocketName: name,
            //             channelName: channel,
            //             param: option.param,
            //             dataFrequencyMs: option.dataFrequencyMs || dataFrequencyMs,
            //         },
            //         sourceType: 'websocktSubscribe',
            //         targetType: 'module',
            //         requestId: 'dreamview',
            //     },
            // } as RequestStreamMessage);
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
                    this.sendSubscriptionMessage(
                        RequestMessageActionEnum.UNSUBSCRIBE_MESSAGE_TYPE,
                        name,
                        channel,
                        option,
                    );
                    // this.mainConnection.sendMessage({
                    //     action: RequestMessageActionEnum.UNSUBSCRIBE_MESSAGE_TYPE,
                    //     type: RequestMessageActionEnum.UNSUBSCRIBE_MESSAGE_TYPE,
                    //     data: {
                    //         name: 'unsubscribe',
                    //         source: 'dreamview',
                    //         info: {
                    //             websocketName: name,
                    //             channelName: channel,
                    //         },
                    //         sourceType: 'websocktSubscribe',
                    //         targetType: 'module',
                    //         requestId: 'unsubscribe',
                    //     },
                    // } as RequestStreamMessage);

                    this.dataSubjects.deleteByExactKey({ name, channel }, (countedSubject) =>
                        countedSubject.complete(),
                    );
                }

                // 如果没有下游订阅者，就取消上游数据源的订阅
                if (this.dataSubjects.countIf((subject) => subject.name === name) === 1) {
                    this.dataSubjects.delete({ name }, (countedSubject) => countedSubject.complete());
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
        if (socket === SocketNameEnum.MAIN) {
            this.mainConnection.sendMessage({ ...request });
        }

        if (socket === SocketNameEnum.PLUGIN) {
            this.pluginConnection.sendMessage({ ...request });
        }
    }
}

export const webSocketManager = new WebSocketManager();
