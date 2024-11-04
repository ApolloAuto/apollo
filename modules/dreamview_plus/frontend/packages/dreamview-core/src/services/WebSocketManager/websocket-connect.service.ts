import { webSocket, WebSocketSubject } from 'rxjs/webSocket';
import { BehaviorSubject, delay, Observable, retryWhen, Subject, take } from 'rxjs';
import Logger from '@dreamview/log';
import MessageQueue from '@dreamview/dreamview-core/src/util/MessageQueue';
import { ConnectionStatusEnum, RequestMessage, RequestStreamMessage, ResponseMessage } from './type';
import { baseURL } from './constant';

const requestIdleCallbackTimeout = 2000;

const logger = Logger.getInstance('WebSocketManager');

export class WebSocketConnection {
    private readonly url: string;

    private socket: WebSocketSubject<RequestMessage<any> | RequestStreamMessage | ResponseMessage<any>>;

    private readonly receivedMessagesSubject = new Subject<ResponseMessage<any>>();

    connectionStatus$ = new BehaviorSubject<ConnectionStatusEnum>(ConnectionStatusEnum.DISCONNECTED);

    private messageQueue = new MessageQueue<RequestMessage<unknown> | RequestStreamMessage>({
        name: 'WebSocketConnection',
        debounceTime: 300,
    });

    constructor(url: string) {
        // 如果url是相对路径，baseUrl
        if (!url.startsWith('ws://') && !url.startsWith('wss://')) {
            this.url = `${baseURL}${url}`;
        } else {
            this.url = url;
        }
        // 监听连接状态 一旦连接成功，就开始消费消息队列
        this.connectionStatus$.subscribe((status) => {
            if (status === ConnectionStatusEnum.CONNECTED) {
                this.consumeMessageQueue();
            }
        });
    }

    connect(retries = 3, retryInterval = 1000) {
        logger.info(`Connecting to ${this.url}`);
        this.connectionStatus$.next(ConnectionStatusEnum.CONNECTING);
        this.socket = webSocket({
            url: this.url,
            openObserver: {
                next: () => {
                    logger.debug(`Connected to ${this.url}`);
                    this.connectionStatus$.next(ConnectionStatusEnum.CONNECTED);
                },
            },
            closeObserver: {
                next: () => {
                    logger.debug(`Disconnected from ${this.url}`);
                    this.connectionStatus$.next(ConnectionStatusEnum.DISCONNECTED);
                },
            },
        });

        this.socket.pipe(retryWhen((errors) => errors.pipe(delay(retryInterval), take(retries)))).subscribe(
            (msg) => {
                this.receivedMessagesSubject.next(msg as ResponseMessage<any>);
            },
            (error) => {
                // handle error
                logger.error(error);
            },
        );

        return this.connectionStatus$;
    }

    isConnected(): boolean {
        logger.debug(`Checking connection status for ${this.url}, status: ${this.connectionStatus$.getValue()}`);
        return this.connectionStatus$.getValue() >= ConnectionStatusEnum.CONNECTED;
    }

    // 断开连接逻辑
    disconnect() {
        if (this.socket) {
            logger.debug(`Disconnecting from ${this.url}`);
            this.socket.complete();
        } else {
            logger.warn('Attempted to disconnect, but socket is not initialized.');
        }
    }

    // 发送消息逻辑
    sendMessage<T>(message: RequestMessage<T> | RequestStreamMessage) {
        this.messageQueue.enqueue(message);
        if (this.isConnected()) {
            logger.debug(`Queueing message to ${this.url}, message: ${JSON.stringify(message, null, 0)}`);
            this.consumeMessageQueue();
        } else {
            logger.debug('Attempted to send message, but socket is not initialized or not connected.');
        }
    }

    private consumeMessageQueue() {
        // 用requestIdleCallback排队执行的函数
        const idleConsume = () => {
            // 仅当浏览器空闲时，才开始处理队列中的消息
            while (!this.messageQueue.isEmpty() && this.isConnected()) {
                const message = this.messageQueue.dequeue();
                if (message) {
                    logger.debug(
                        `Sending message from queue to ${this.url}, message: ${JSON.stringify(message, null, 0)}`,
                    );
                    this.socket.next(message);
                }
            }
        };

        // 首次调用requestIdleCallback来开始处理消息队列
        requestIdleCallback(idleConsume, { timeout: requestIdleCallbackTimeout });
    }

    get receivedMessages$(): Observable<ResponseMessage<any>> {
        return this.receivedMessagesSubject.asObservable();
    }
}
