import { webSocket, WebSocketSubject } from 'rxjs/webSocket';
import { BehaviorSubject, delay, Observable, retryWhen, Subject, take } from 'rxjs';
import Logger from '@dreamview/log';
import { ConnectionStatusEnum, RequestMessage, RequestStreamMessage, ResponseMessage } from './type';

const logger = Logger.getInstance('WebSocketManager');

export class WebSocketConnection {
    private readonly url: string;

    private socket: WebSocketSubject<RequestMessage<any> | RequestStreamMessage | ResponseMessage<any>>;

    private readonly receivedMessagesSubject = new Subject<ResponseMessage<any>>();

    connectionStatus$ = new BehaviorSubject<ConnectionStatusEnum>(ConnectionStatusEnum.DISCONNECTED);

    constructor(url: string) {
        this.url = url;
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
        if (this.socket) {
            logger.debug(`Sending message to ${this.url}, message: ${JSON.stringify(message, null, 0)}`);
            this.socket.next(message);
        } else {
            logger.error('Attempted to send message, but socket is not initialized.');
        }
    }

    get receivedMessages$(): Observable<ResponseMessage<any>> {
        return this.receivedMessagesSubject.asObservable();
    }
}
