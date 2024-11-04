// task type
import { DispatchTaskOption } from '../services/WebSocketManager';

export interface TaskInternal<TPayload> {
    id: string;
    type: string;
    payload: TPayload;
    transferList?: Transferable[];
    priority: number;
    option?: DispatchTaskOption;
}

export type Task<TPayload> = Omit<TaskInternal<TPayload>, 'id' | 'priority'> & {
    id?: string;
    priority?: number;
};

export interface WorkerResponse<T> {
    success: boolean;
    id: string;
    result?: T;
    error?: string;
}

interface BasicWorker {
    postMessage(message: any, transferList?: Transferable[]): void;
    terminate(): void;
    onmessage: ((this: Worker, ev: MessageEvent) => any) | null;
    onerror: ((this: Worker, ev: ErrorEvent) => any) | null;
    onmessageerror: ((this: Worker, ev: MessageEvent) => any) | null;
    addEventListener: (
        type: string,
        listener: EventListenerOrEventListenerObject,
        options?: boolean | AddEventListenerOptions,
    ) => void;
    removeEventListener: (
        type: string,
        listener: EventListenerOrEventListenerObject,
        options?: boolean | EventListenerOptions,
    ) => void;
    dispatchEvent: (event: Event) => boolean;
}

export abstract class AbstractWorkerWrapper implements BasicWorker {
    protected worker: BasicWorker;

    // 实现基本方法
    postMessage(message: any, transfer?: Transferable[]): void {
        this.worker.postMessage(message, transfer);
    }

    terminate(): void {
        // 在这里可以添加清理逻辑
        this.worker.terminate();
    }

    get onmessage(): ((this: Worker, ev: MessageEvent) => any) | null {
        return this.worker.onmessage;
    }

    set onmessage(value: ((this: Worker, ev: MessageEvent) => any) | null) {
        this.worker.onmessage = value;
    }

    get onerror(): ((this: Worker, ev: ErrorEvent) => any) | null {
        return this.worker.onerror;
    }

    set onerror(value: ((this: Worker, ev: ErrorEvent) => any) | null) {
        this.worker.onerror = value;
    }

    get onmessageerror(): ((this: Worker, ev: MessageEvent) => any) | null {
        return this.worker.onmessageerror;
    }

    set onmessageerror(value: ((this: Worker, ev: MessageEvent) => any) | null) {
        this.worker.onmessageerror = value;
    }

    addEventListener(
        type: string,
        listener: EventListenerOrEventListenerObject,
        options?: boolean | AddEventListenerOptions,
    ): void {
        this.worker.addEventListener(type, listener, options);
    }

    removeEventListener(
        type: string,
        listener: EventListenerOrEventListenerObject,
        options?: boolean | EventListenerOptions,
    ): void {
        this.worker.removeEventListener(type, listener, options);
    }

    dispatchEvent(event: Event): boolean {
        return this.worker.dispatchEvent(event);
    }
}

export interface IWorker<TPayload> {
    worker: AbstractWorkerWrapper;
    isIdle: boolean;
    postMessage: (task: Task<TPayload>, transferList?: Transferable[]) => void;
    onmessage: ((this: IWorker<TPayload>, ev: MessageEvent<Task<any>>) => any) | null;
    terminate: () => void;
    lastUsedTime?: number;
    setIdle: (isIdle: boolean) => void;
}
