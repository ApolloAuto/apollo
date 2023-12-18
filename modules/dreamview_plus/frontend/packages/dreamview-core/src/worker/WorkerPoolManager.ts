import Logger from '@dreamview/log';
import { generate } from 'short-uuid';
import { IWorker, Task, TaskInternal, WorkerResponse } from './type';
import { WorkerFactory } from './WorkerFactory';
import MessageQueue from '../util/MessageQueue';

export interface WorkerPoolManagerConfig<TPayload> {
    name: string;
    size?: number;
    performanceMode?: boolean;
    workerFactory: WorkerFactory<TPayload>;
}

export default class WorkerPoolManager<TPayload> {
    private static totalWorkerCount = 0;

    public static getTotalWorkerCount(): number {
        return WorkerPoolManager.totalWorkerCount;
    }

    private logger: Logger;

    private maxWorkerSize = 20;

    private minWorkerSize = 1;

    private pidController: { integral: number; previousError: number; Kp: number; Ki: number; Kd: number } = {
        integral: 0,
        previousError: 0,
        // 任务数量增加时快速增加线程数
        Kp: 0.2,
        // 处理长期的小误差
        Ki: 0.01,
        // 预测未来趋势，平滑系统响应。
        Kd: 0.05,
    };

    set workerSize(size: number) {
        this.adjustWorkerSize(size);
    }

    private resizeTimeoutId: number | null = null;

    private pool: IWorker<TPayload>[] = [];

    private queue: MessageQueue<TaskInternal<TPayload>>;

    private handleEnqueue = (task: TaskInternal<TPayload>) => {
        this.logger.debug(`Enqueue task: ${this.queue.size},${task.id}`);
        this.adjustWorkerSizeWithPID();
    };

    private handleDequeue = (task: TaskInternal<TPayload>) => {
        this.logger.debug(`Dequeue task: ${this.queue.size},${task.id}`);
        if (this.queue.size === 0) {
            this.terminateIdleWorkers();
        }
    };

    private readonly workerFactory: WorkerFactory<TPayload>;

    private taskResolvers: Map<
        string,
        { resolve: (res: WorkerResponse<unknown>) => void; reject: (err: Error) => void }
    > = new Map();

    constructor(private readonly config: WorkerPoolManagerConfig<TPayload>) {
        const { workerFactory, name } = config;
        this.logger = Logger.getInstance(`ThreadPoolManager-${name}`);
        // window.setLogLevel(`ThreadPoolManager-${name}`, 'debug');
        this.workerFactory = workerFactory;
        this.queue = new MessageQueue({
            name,
            onEnqueue: this.handleEnqueue.bind(this),
            onDequeue: this.handleDequeue.bind(this),
        });
        this.init();
    }

    init(): void {
        for (let i = 0; i < this.minWorkerSize / 2; i += 1) {
            this.createWorker();
        }
        const { size } = this.config;
        if (size) {
            this.workerSize = size;
        }
    }

    private sendTaskToWorker(worker: IWorker<TPayload>, taskInternal: TaskInternal<TPayload>): void {
        if (taskInternal.transferList) {
            worker.postMessage(taskInternal, taskInternal.transferList);
        } else {
            worker.postMessage(taskInternal);
        }
        worker.setIdle(false);
    }

    dispatchTask(task: Task<TPayload>): Promise<WorkerResponse<unknown>> {
        return new Promise((resolve, reject) => {
            const id = `${task.type}-${generate()}`;
            const worker = this.getAvailableWorker();
            const taskInternal: TaskInternal<TPayload> = { id, priority: 0, ...task };
            if (worker) {
                this.sendTaskToWorker(worker, taskInternal);
            } else if (this.pool.length < this.maxWorkerSize) {
                const newWorker = this.createWorker();
                this.sendTaskToWorker(newWorker, taskInternal);
            } else {
                // fixme: 实现优先级逻辑
                this.queue.enqueue(taskInternal);
            }
            this.taskResolvers.set(id, { resolve, reject });
        });
    }

    private getAvailableWorker(): IWorker<TPayload> | null {
        return this.pool.find((worker) => worker.isIdle) || null;
    }

    private createWorker(): IWorker<TPayload> {
        const iworker = this.workerFactory.createWorker();
        iworker.worker.onmessage = (e) => {
            this.handleWorkerMessage(iworker, e);
            this.dispatchQueuedTasks();
        };
        this.pool.push(iworker);
        WorkerPoolManager.totalWorkerCount += 1;
        return iworker;
    }

    private dispatchQueuedTasks(): void {
        while (this.queue.size > 0 && this.getAvailableWorker()) {
            const task = this.queue.dequeue();
            const idleWorker = this.getAvailableWorker();
            if (idleWorker) {
                this.sendTaskToWorker(idleWorker, task);
            }
        }
    }

    private handleWorkerMessage(worker: IWorker<TPayload>, e: MessageEvent): void {
        worker.setIdle(true);
        const { id, success, result, error } = e.data as WorkerResponse<unknown>;
        const taskResolver = this.taskResolvers.get(id);
        if (taskResolver) {
            try {
                if (success) {
                    taskResolver.resolve({ success, id, result });
                } else {
                    taskResolver.reject(new Error(error));
                }
            } catch (err: any) {
                this.logger.error(err);
                taskResolver.reject(new Error(err));
            }
            this.taskResolvers.delete(id);
        }
    }

    private adjustWorkerSizeWithPID(): void {
        const error = this.queue.size; // 理想队列长度为0
        this.pidController.integral += error;
        const derivative = error - this.pidController.previousError;

        const adjustment =
            this.pidController.Kp * error +
            this.pidController.Ki * this.pidController.integral +
            this.pidController.Kd * derivative;

        // 判断阈值，避免小的队列变动立即触发线程数调整
        if (Math.abs(adjustment) > 2) {
            this.workerSize = Math.round(this.workerSize + adjustment);
            this.workerSize = Math.min(Math.max(this.workerSize, this.minWorkerSize), this.maxWorkerSize);
        }

        this.pidController.previousError = error;
    }

    public adjustWorkerSize(size: number): void {
        // 如果有现有的延时操作，则先清除
        if (this.resizeTimeoutId !== null) {
            clearTimeout(this.resizeTimeoutId);
            this.resizeTimeoutId = null;
        }

        if (size < this.pool.length) {
            // 减少workers数量
            const idleWorker = this.pool.find((worker) => worker.isIdle);
            if (idleWorker) {
                idleWorker.terminate();
                this.pool = this.pool.filter((worker) => worker !== idleWorker);
                WorkerPoolManager.totalWorkerCount -= 1;
            }

            // 设置3秒后再次调用此方法，直到达到目标size
            if (this.pool.length > size) {
                this.resizeTimeoutId = setTimeout(() => this.adjustWorkerSize(size), 3000) as unknown as number;
            }
        } else if (size > this.pool.length) {
            while (this.pool.length < size) {
                this.createWorker();
            }
            this.dispatchQueuedTasks();
        }
    }

    // 遍历所有worker，执行回调
    forEach(callback: (worker: IWorker<TPayload>) => void): void {
        this.pool.forEach(callback);
    }

    // 将超时worker直接重新使用
    resetIdleWorkers(): void {
        const idleThreshold = 60000;
        const now = Date.now();
        this.pool.forEach((worker) => {
            if (worker.isIdle && now - worker.lastUsedTime > idleThreshold) {
                const task = this.queue.dequeue();
                if (task) {
                    this.sendTaskToWorker(worker, task);
                } else {
                    worker.setIdle(false);
                }
            }
        });
    }

    // 清理空闲的worker
    terminateIdleWorkers(): void {
        const idleThreshold = 10000;
        const now = Date.now();
        this.pool = this.pool.filter((worker) => {
            const { isIdle, lastUsedTime } = worker;
            const idleTime = now - lastUsedTime;
            if (isIdle && idleTime > idleThreshold) {
                worker.terminate();
                WorkerPoolManager.totalWorkerCount -= 1;
                return false;
            }
            return true;
        });
    }

    // 销毁所有worker
    terminateAllWorkers(): void {
        this.pool.forEach((worker) => worker.terminate());
        this.pool = [];
        WorkerPoolManager.totalWorkerCount = 0;
    }

    visualize(): void {
        const activeWorkerCount = this.pool.filter((worker) => !worker.isIdle).length;
        const queuedTasksCount = this.queue.size;
        const allWorkers = WorkerPoolManager.getTotalWorkerCount();

        this.logger.info('[WorkerPoolManager Status]');
        this.logger.info('[Active Workers]/[Current Workers]/[All Workers]:');
        this.logger.info(` ${activeWorkerCount} / ${this.pool.length} / ${allWorkers}`);
        this.logger.info(`Queued Tasks: ${queuedTasksCount}`);
    }
}
