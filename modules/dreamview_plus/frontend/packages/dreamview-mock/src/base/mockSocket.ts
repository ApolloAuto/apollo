import WebSocket from 'ws';
import fs from 'fs';
import * as path from 'path';
import { from, Subject, interval, Subscription, of, Observable } from 'rxjs';
import { concatMap, take, repeat } from 'rxjs/operators';
import http from 'http';
import {
    Metadata, RequestMessage,
    RequestMessageActionEnum, RequestStreamMessage, ResponseMessage,
    ResponseMessageActionEnum,
} from '@dreamview/dreamview-core/src/services/WebSocketManager/type';
import { MetadataMessage } from '@dreamview/dreamview-core/src/services/apiType';
import { config } from '@dreamview/dreamview-core/src/services/WebSocketManager/constant';

export interface Mock {
    dataName?: string;
    protoPath?: string;
    requestName?: string;
    websocketInfo?: {
        websocketName: string;
        websocketPipe: string;
    },
    response?: <T>(data: RequestMessage<T>) => Promise<WebSocket.Data>;
    channels?: string[];
    differentForChannels?: boolean;
    subscribe?: boolean;
    interval?: number;
    delay?: number;
    subscription?: Subscription;
    // 对于请求响应接口使用mockType关联controller
    mockType?: string;
    // 对于HMIAction接口使用mockName关联controller,mockType统一为HMIAction
    mockName?: string;
}

const RootPath = path.resolve(__dirname, '../../');

class MockSocketServer {

    private server: http.Server;

    private wss: WebSocket.Server;

    private mocks: Record<string, { subject: Subject<any>, socket: WebSocket.Server } & Mock> = {};

    private dataIntervals: {[dataName: string]: Observable<number>} = {};

    constructor(port: number) {
        this.server = http.createServer();
        this.wss = new WebSocket.Server({ noServer: true });

        this.server.on('upgrade', (request, socket, head) => {
            const pathname = new URL(request.url || '', `http://localhost:${port}`).pathname;

            console.log(`Mock socket ${pathname} connected.`);

            if (config.mainUrl.includes(pathname)) {
                this.wss.handleUpgrade(request, socket, head, (ws) => {
                    this.wss.emit('connection', ws, request);
                });
            } else if (config.pluginUrl.includes(pathname)) {
                this.wss.handleUpgrade(request, socket, head, (ws) => {
                    this.wss.emit(`${config.pluginUrl}connection`, ws, request);
                });
            } else if (pathname.substring(1) in this.mocks) {
                console.log(`Mock socket ${pathname.substring(1)} connected.`);
                const mockSocket = this.mocks[pathname.substring(1)].socket;
                mockSocket.handleUpgrade(request, socket, head, (ws) => {
                    mockSocket.emit('connection', ws, request);
                });
            } else {
                socket?.destroy();
            }
        });

        this.server.listen(port, () => {
            console.log(`WebSocket server started on port ${port}`);
        });

        this.wss.on('connection', (ws) => {

            ws.send(JSON.stringify({
                    action: ResponseMessageActionEnum.METADATA_MESSAGE_TYPE,
                    data: {
                        name: 'metadata',
                        source: 'dreamview',
                        info: {
                            code: 0,
                            message: 'success',
                            data: {
                                dataHandlerInfo: Object.keys(this.mocks)
                                    .filter(mockName => this.mocks[mockName].subscribe)
                                    .reduce<Metadata['dataHandlerInfo']>((acc, key) => {
                                        acc[key] = {
                                            dataName: key,
                                            protoPath: key,
                                            protoDesc: key,
                                            requestName: key,
                                            channels: this.mocks[key].channels ?? [],
                                            differentForChannels: this.mocks[key].differentForChannels ?? false,
                                            websocketInfo: {
                                                websocketName: key,
                                                websocketPipe: key,
                                            },
                                        };
                                        return acc;
                                    }, {}),
                            },
                        },
                        target: 'dreamview',
                        sourceType: 'dreamview',
                        targetType: 'dreamview',
                        requestId: 'dreamview',
                    },
                } as MetadataMessage),
            );

            ws.on('message', (data) => {
                if (data instanceof Buffer) {
                    const stringMessage = data.toString();
                    const parsedMessage = JSON.parse(stringMessage) as RequestStreamMessage | RequestMessage<any>;
                    const action = parsedMessage?.action;
                    console.log('Received message:', parsedMessage);

                    if ([RequestMessageActionEnum.SUBSCRIBE_MESSAGE_TYPE, RequestMessageActionEnum.UNSUBSCRIBE_MESSAGE_TYPE].includes(action)) {
                        const dataName = parsedMessage?.data.info.websocketName;
                        const dataFrequencyMs = parsedMessage?.data.info?.dataFrequencyMs;
                        const channelName = parsedMessage?.data.info?.channelName;
                        const mocksTag = `${dataName}${channelName ? `_${channelName}` : ''}`;
                        if (action === RequestMessageActionEnum.SUBSCRIBE_MESSAGE_TYPE &&
                            dataName in this.mocks &&
                            this.mocks[dataName].subscribe
                        ) {
                            this.mocks[mocksTag].subscription = this.streamFileLoader(dataName, dataFrequencyMs).subscribe((data) => {
                                this.triggerEvent(dataName, data);
                            });
                        }


                        if (action === RequestMessageActionEnum.UNSUBSCRIBE_MESSAGE_TYPE && dataName in this.mocks && this.mocks[dataName].subscription) {

                            this.mocks[mocksTag].subscription?.unsubscribe();
                        }
                    }

                    if (action == RequestMessageActionEnum.REQUEST_MESSAGE_TYPE) {
                        console.log('Received request:', parsedMessage);
                        this.handleMessage(parsedMessage, parsedMessage, ws);
                    }
                }
            });
        });

        this.wss.on(`${config.pluginUrl}connection`, (ws) => {
            console.log(`Mock socket ${config.pluginUrl} connected.`);
            ws.on('message', (data: WebSocket.RawData) => {
                if (data instanceof Buffer) {
                    const stringMessage = data.toString();
                    const parsedMessage = JSON.parse(stringMessage) as RequestStreamMessage | RequestMessage<any>;
                    const action = parsedMessage?.action;
                    console.log('Received message:', parsedMessage);

                    if (action == RequestMessageActionEnum.REQUEST_MESSAGE_TYPE) {
                        console.log('Received request:', parsedMessage);
                        this.handleMessage(parsedMessage, parsedMessage, ws);
                    }
                }
            });
        });
    }

    addMock(mock: Mock) {
        const subject = new Subject();
        const socket = new WebSocket.Server({ noServer: true });
        if (mock.subscribe && mock.dataName) {
            subject.subscribe((data) => {
                if (mock.subscribe) {
                    // For subscriptions, send data to all subscribers when subject emits
                    socket.clients.forEach((client) => {
                        if (client.readyState === WebSocket.OPEN) {
                            if (data instanceof Buffer) {
                                client.send(data);
                            }
                        }
                    });
                }
            });
            this.mocks[mock.dataName] = { subject, socket, ...mock };

            if (mock.differentForChannels) {
                mock.channels?.forEach(channel => {
                    this.mocks[`${mock.dataName}_${channel}`] = { subject, socket };
                });
            }
        }

        if (mock.response) {
            const response = this.wrapResponse(mock.response);
            this.mocks[`${mock.mockType}-${mock.mockName}`] = { subject, socket, ...mock, response };
        }
    }

    private streamFileLoader(
        dataName: string,
        dataFrequencyMs = 100,
        dir: string = path.join(RootPath, './recordings'),
        prefix = 'sensor_rgb',
    ) {

        const files = fs
            .readdirSync(dir)
            .filter((filename) => filename.startsWith(`${prefix}-${dataName.replace('/', '-')}`))
            .map((filename) => path.join(dir, filename))
            .sort();

        const fileCache: Record<string, Buffer> = {};

        if (!this.dataIntervals[dataName]) {
            this.dataIntervals[dataName] = interval(dataFrequencyMs)
        }

        // Create an interval observable that emits every 1/frequency seconds
        const interval$ = this.dataIntervals[dataName];

        return interval$
            .pipe(
                take(files.length), // Take only as many ticks as there are files
                concatMap((i) => {
                    if (fileCache[files[i]]) {
                        return of(fileCache[files[i]]);
                    } else {
                        const data = fs.readFileSync(files[i]);
                        fileCache[files[i]] = data;
                        return of(data);
                    }
                }), // For each tick, map to a file
                repeat(), // Repeat the sequence indefinitely
            );
    }

    private handleMessage<T>(parsedMessage: RequestMessage<any>, data: RequestMessage<T>, ws: WebSocket) {
        // 对于请求响应接口使用mockType关联controller
        // 对于HMIAction接口使用mockName关联controller,mockType统一为HMIAction
        const type = parsedMessage?.type;
        const name = parsedMessage?.data?.name;

        const filterMocks = Object.keys(this.mocks).filter((mockName) =>
            this.mocks[mockName].mockType === type && this.mocks[mockName].mockName === name,
        );


        if (filterMocks.length > 0) {
            const mock = this.mocks[filterMocks[0]] ?? {};
            const responseData = mock.response && mock.response(data);
            if (responseData) {
                const responseObservable = from(responseData);
                responseObservable.subscribe(
                    (responseData) => ws.send(responseData),
                    (error) => console.error('Error in response function:', error),
                );
            }
        }
    }

    private wrapResponse<T>(func: (data: RequestMessage<T>) => Promise<any>) {
        return async (data: RequestMessage<T>) => {
            try {
                const result = await func(data);
                const response: ResponseMessage<any> = {
                    action: ResponseMessageActionEnum.RESPONSE_MESSAGE_TYPE,
                    data: {
                        name: data.data.name,
                        source: 'dreamview',
                        info: {
                            code: 0,
                            message: 'success',
                            data: result,
                        },
                        target: 'dreamview',
                        sourceType: 'dreamview',
                        targetType: 'dreamview',
                        requestId: data.data.requestId,
                    },
                };
                return JSON.stringify(response);
            } catch (e) {
                const errorResponse: ResponseMessage<null> = {
                    action: ResponseMessageActionEnum.RESPONSE_MESSAGE_TYPE,
                    data: {
                        name: data.data.name,
                        source: 'dreamview',
                        info: {
                            code: -1,
                            message: 'fail',
                            data: null,
                        },
                        target: 'dreamview',
                        sourceType: 'dreamview',
                        targetType: 'dreamview',
                        requestId: data.data.requestId,
                    },
                };
                return JSON.stringify(errorResponse);
            }
        };
    }

    triggerEvent(dataName: string, data: any) {
        const mock = this.mocks[dataName];
        if (mock) {
            mock.subject.next(data);
        }
    }
}

export default MockSocketServer;
