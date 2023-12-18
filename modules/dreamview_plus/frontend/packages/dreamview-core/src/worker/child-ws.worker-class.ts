/* eslint-disable import/no-webpack-loader-syntax */
// @ts-ignore
import ChildWsWorker from 'worker-loader!./childWs.worker';
import { Observable, Subject } from 'rxjs';
import Logger from '@dreamview/log';
import { StreamDataNames } from '@dreamview/dreamview-core/src/services/api/types';
import {
    WorkerMessage,
    SocketState,
    WorkerMessageType,
    isMessageType,
    MetadataItem,
} from '@dreamview/dreamview-core/src/services/WebSocketManager/type';
import { AbstractWorkerWrapper } from './type';

const logger = Logger.getInstance('ChildWsWorkerClass');
class ChildWsWorkerClass extends AbstractWorkerWrapper {
    private socketMessageSubject: Subject<WorkerMessage<WorkerMessageType>> = new Subject();

    state: SocketState = 'CLOSED';

    constructor(private name: StreamDataNames, private url: string) {
        super();
        this.worker = new ChildWsWorker();
        this.onmessage = this.handleWorkerMessage.bind(this);
    }

    private handleWorkerMessage(event: MessageEvent<WorkerMessage<WorkerMessageType>>) {
        const message = event.data;

        if (isMessageType(message, 'SOCKET_MESSAGE')) {
            this.socketMessageSubject.next(message);
            return;
        }

        if (isMessageType(message, 'SOCKET_OPEN')) {
            this.state = 'OPEN';
            const endTimestamp = performance.now();
            // @ts-ignore
            const startTimestamp = message?.payload?.performance?.startTimestamp;
            if (startTimestamp) {
                logger.debug(`Message from ${this.name} took ${endTimestamp - startTimestamp}ms to SOCKET_OPEN`);
            }
        }

        if (isMessageType(message, 'SOCKET_CLOSED')) {
            this.state = 'CLOSED';
        }
    }

    get socketMessage$(): Observable<WorkerMessage<WorkerMessageType>> {
        return this.socketMessageSubject.asObservable();
    }

    /**
     * 连接子 WebSocket
     */
    connect() {
        this.postMessage({ type: 'INIT_SOCKET', payload: { name: this.name, url: this.url } });
        return this;
    }

    /**
     * 断开子 WebSocket
     */
    disconnect() {
        this.postMessage({ type: 'DESTROY_SOCKET' });
        return this;
    }

    /**
     * 同步metadata用于反序列化数据
     */
    syncMetadata(metadata: MetadataItem[]) {
        this.postMessage({ type: 'SOCKET_METADATA', payload: metadata });
        return this;
    }
}

export default ChildWsWorkerClass;
