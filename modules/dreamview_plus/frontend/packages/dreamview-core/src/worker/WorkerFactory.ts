import { AbstractWorkerWrapper, IWorker } from './type';

export class WorkerFactory<TPayload> {
    constructor(private readonly generator: () => AbstractWorkerWrapper) {}

    public createWorker(): IWorker<TPayload> {
        const worker = this.generator();
        const workerObject: IWorker<TPayload> = {
            worker,
            isIdle: true,
            lastUsedTime: Date.now(),
            postMessage: worker.postMessage.bind(worker),
            onmessage: null,
            terminate: worker.terminate.bind(worker),
            // 实现setIdle方法
            setIdle: (isIdle: boolean) => {
                workerObject.isIdle = isIdle;
                if (isIdle) {
                    workerObject.lastUsedTime = Date.now();
                }
            },
        };

        return workerObject;
    }
}
