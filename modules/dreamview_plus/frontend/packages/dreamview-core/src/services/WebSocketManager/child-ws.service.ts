/* eslint-disable import/no-webpack-loader-syntax */
// @ts-ignore
import ChildWsDeserializerWorker from 'worker-loader!./childWsDeserializerWorker';
import { Observable, Subject } from 'rxjs';
import { WorkerMessage, SocketState, WorkerMessageType, isMessageType } from './type';
import { StreamDataNames } from '../api/types';

class ChildWsService {
    private worker: Worker;

    private socketMessageSubject: Subject<WorkerMessage<WorkerMessageType>> = new Subject();

    state: SocketState = 'CLOSED';

    constructor(private name: StreamDataNames, private url: string) {
        this.worker = new ChildWsDeserializerWorker();
        this.worker.onmessage = this.handleWorkerMessage.bind(this);
    }

    private handleWorkerMessage(event: MessageEvent<WorkerMessage<WorkerMessageType>>) {
        const message = event.data;

        if (isMessageType(message, 'SOCKET_MESSAGE')) {
            this.socketMessageSubject.next(message);
            return;
        }

        if (isMessageType(message, 'SOCKET_OPEN')) {
            this.state = 'OPEN';
        }

        if (isMessageType(message, 'SOCKET_CLOSED')) {
            this.state = 'CLOSED';
        }
    }

    get socketMessage$(): Observable<WorkerMessage<WorkerMessageType>> {
        return this.socketMessageSubject.asObservable();
    }

    send(message: WorkerMessage<WorkerMessageType>) {
        this.worker.postMessage(message);
        return this;
    }

    /**
     * 连接子 WebSocket
     */
    connect() {
        this.send({ type: 'INIT_SOCKET', payload: { name: this.name, url: this.url } });
        return this;
    }

    /**
     * 断开子 WebSocket
     */
    disconnect() {
        this.send({ type: 'DESTROY_SOCKET' });
        return this;
    }
}

export default ChildWsService;
