/* eslint-disable import/no-webpack-loader-syntax */
import { webSocket, WebSocketSubject } from 'rxjs/webSocket';
import { from, filter, finalize, Observable, Subject, map, BehaviorSubject } from 'rxjs';
import Logger from '@dreamview/log';
// @ts-ignore
import DeserializerWorker from 'worker-loader!./deserializerWorker';
import { isEqual } from 'lodash';
import { WebSocketConnection } from './websocket-connect.service';
import { genereateRequestId } from '../../util/genereateRequestId';
import { config } from './constant';
import { ChildWebSocketSubjectConfig } from './deserializerMethods';
import {
    ConnectionStatusEnum,
    RequestMessageActionEnum,
    ResponseMessageActionEnum,
    MetadataItem,
    SocketNameEnum,
    RequestMessage,
    ResponseMessage,
    RequestStreamMessage,
    StreamMessageData,
} from './type';

import { HandleMessageType } from '../models/response-message.model';
import { RequestDataType } from '../models/request-message.model';
import CountedSubject from '../../util/CountedSubject';
import MultiKeyMap from '../../util/MultiKeyMap';
import { Emptyable } from '../../util/similarFunctions';

const logger = Logger.getInstance('WebSocketManager');

const dataFrequencyMs = 100;

export class WebSocketManager {
    private readonly mainConnection: WebSocketConnection;

    private readonly pluginConnection: WebSocketConnection;

    private activeSockets: { [key: string]: WebSocketSubject<any> } = {};

    private metadata: MetadataItem[] = [];

    private metadataSubject: BehaviorSubject<MetadataItem[]> = new BehaviorSubject<MetadataItem[]>([]);

    private dataSubjects: MultiKeyMap<
        { name: string; channel?: Emptyable<string> },
        CountedSubject<StreamMessageData<unknown> | unknown>
    > = new MultiKeyMap();

    private responseResolvers: {
        [requestId: string]: {
            resolver: (response: any) => void;
            shouldDelete: boolean;
        };
    } = {};

    constructor(mainUrl: string = config.mainUrl, pluginUrl: string = config.pluginUrl) {
        this.mainConnection = new WebSocketConnection(mainUrl);
        this.pluginConnection = new WebSocketConnection(pluginUrl);
        this.mainConnection.receivedMessages$.subscribe((msg) =>
            this.handleMessage(<HandleMessageType>msg, SocketNameEnum.MAIN),
        );
        this.pluginConnection.receivedMessages$.subscribe((msg) =>
            this.handleMessage(<HandleMessageType>msg, SocketNameEnum.PLUGIN),
        );
    }

    handleMessage(msg: HandleMessageType, socketName: SocketNameEnum) {
        if (!(msg?.data?.info?.code === 0)) {
            return;
        }
        logger.debug(`Received message from ${socketName}, message: ${JSON.stringify(msg, null, 0)}`);

        if (msg.data.info.code !== 0) {
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
                this.responseResolvers[msg.data.requestId].resolver(msg);
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
        Object.entries(this.activeSockets).forEach(([name, socket]) => {
            socket.complete();
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

        const childSocket = webSocket(
            ChildWebSocketSubjectConfig(`${config.baseURL}/${metadata.websocketInfo.websocketName}`),
        );
        this.activeSockets[name] = childSocket;

        const worker = new DeserializerWorker();

        worker.onmessage = (event: MessageEvent<StreamMessageData<unknown>>) => {
            const streamMessageData = event.data;
            this.dataSubjects.getByExactKey({ name })?.next(streamMessageData);
        };

        childSocket.subscribe((data: ArrayBuffer) => {
            worker.postMessage({ data, name }, [data]);
        });
    }

    /**
     * 订阅指定name的数据
     * @param name 数据名
     */
    public subscribeToData<T>(name: string): CountedSubject<T> {
        if (this.dataSubjects.getByExactKey({ name }) === undefined) {
            this.dataSubjects.set({ name }, new CountedSubject(name));
            this.connectChildSocket(name);
            this.mainConnection.sendMessage({
                action: RequestMessageActionEnum.SUBSCRIBE_MESSAGE_TYPE,
                type: RequestMessageActionEnum.SUBSCRIBE_MESSAGE_TYPE,
                data: {
                    name: 'subscribe',
                    source: 'dreamview',
                    info: {
                        websocketName: name,
                        dataFrequencyMs,
                    },
                    sourceType: 'websocktSubscribe',
                    targetType: 'module',
                    requestId: 'dreamview',
                },
            } as RequestStreamMessage);
        }

        const subject$ = this.dataSubjects.getByExactKey({ name }) as CountedSubject<StreamMessageData<unknown>>;

        return <CountedSubject<T>>subject$.pipe(
            map((data) => data?.data as T),
            finalize(() => {
                const subscribersCount = subject$.count;
                const isCompleted = subject$.completed;
                if (isCompleted) return;
                if (subscribersCount === 0) {
                    this.mainConnection.sendMessage({
                        action: RequestMessageActionEnum.UNSUBSCRIBE_MESSAGE_TYPE,
                        type: RequestMessageActionEnum.UNSUBSCRIBE_MESSAGE_TYPE,
                        data: {
                            name: 'unsubscribe',
                            source: 'dreamview',
                            info: {
                                websocketName: name,
                            },
                            sourceType: 'websocktSubscribe',
                            targetType: 'module',
                            requestId: 'unsubscribe',
                        },
                    } as RequestStreamMessage);

                    this.dataSubjects.delete({ name }, (countedSubject) => countedSubject.complete());
                    this.activeSockets[name].complete();
                    delete this.activeSockets[name];
                }
            }),
        );
    }

    /**
     * 订阅指定name、channel的数据
     * @param name 数据名
     * @param channel 频道名
     */
    public subscribeToDataWithChannel<T>(name: string, channel: string): CountedSubject<T> {
        if (this.dataSubjects.getByExactKey({ name }) === undefined) {
            // 初始化上游数据源
            this.dataSubjects.set({ name }, new CountedSubject(name));
            this.connectChildSocket(name);
        }

        if (this.dataSubjects.getByExactKey({ name, channel }) === undefined) {
            this.mainConnection.sendMessage({
                action: RequestMessageActionEnum.SUBSCRIBE_MESSAGE_TYPE,
                type: RequestMessageActionEnum.SUBSCRIBE_MESSAGE_TYPE,
                data: {
                    name: 'subscribe',
                    source: 'dreamview',
                    info: {
                        websocketName: name,
                        channelName: channel,
                        dataFrequencyMs,
                    },
                    sourceType: 'websocktSubscribe',
                    targetType: 'module',
                    requestId: 'dreamview',
                },
            } as RequestStreamMessage);
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
                    this.mainConnection.sendMessage({
                        action: RequestMessageActionEnum.UNSUBSCRIBE_MESSAGE_TYPE,
                        type: RequestMessageActionEnum.UNSUBSCRIBE_MESSAGE_TYPE,
                        data: {
                            name: 'unsubscribe',
                            source: 'dreamview',
                            info: {
                                websocketName: name,
                                channelName: channel,
                            },
                            sourceType: 'websocktSubscribe',
                            targetType: 'module',
                            requestId: 'unsubscribe',
                        },
                    } as RequestStreamMessage);

                    this.dataSubjects.deleteByExactKey({ name, channel }, (countedSubject) =>
                        countedSubject.complete(),
                    );
                }

                // 如果没有下游订阅者，就取消上游数据源的订阅
                if (this.dataSubjects.countIf((subject) => subject.name === name) === 1) {
                    this.dataSubjects.delete({ name }, (countedSubject) => countedSubject.complete());
                    this.activeSockets[name]?.complete();
                    delete this.activeSockets[name];
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

        return new Promise((resolve) => {
            this.responseResolvers[requestId] = {
                resolver: resolve,
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
