import DoublyLinkedList from '@dreamview/dreamview-core/src/util/DoublyLinkedList';
import Logger from '@dreamview/log';
import * as process from 'process';

interface MessageQueueConfig<T> {
    debounceTime?: number;
    // fixme: 忽略容量
    // capacity?: number;
    name: string;
    onEnqueue?: (message: T) => void;
    onDequeue?: (message: T) => void;
    maxLen?: number;
}

export default class MessageQueue<T> {
    private readonly queue: DoublyLinkedList<T>;

    private messageTimestamps: { [id: string]: number } = {};

    private readonly name: string;

    private readonly logger: Logger;

    private onEnqueue?: (message: T) => void;

    private onDequeue?: (message: T) => void;

    private maxLen: number;

    private enqueueCounter = 0;

    private readonly checkInterval = 100;

    constructor(private readonly config: MessageQueueConfig<T>) {
        this.name = config.name;
        this.logger = Logger.getInstance(`MessageQueue-${this.name}`);
        this.onEnqueue = config.onEnqueue;
        this.onDequeue = config.onDequeue;

        const { debounceTime = 0, maxLen = 1000 } = config;
        this.maxLen = maxLen;
        this.queue = new DoublyLinkedList<T>();
        if (debounceTime > 0) {
            setInterval(() => this.cleanup(), debounceTime);
        }
    }

    /**
     * Add a message to the queue.
     * @param message The message to be added.
     * @param prepend Whether to add the message to the head of the queue.
     */
    enqueue(message: T, prepend = false) {
        const { debounceTime = 0 } = this.config;
        if (debounceTime > 0) {
            const messageId = this.getMessageId(message);
            const now = Date.now();
            if (messageId in this.messageTimestamps && now - this.messageTimestamps[messageId] < debounceTime) {
                this.logger.debug(`Duplicate message with id ${messageId} ignored.`);
                return this;
            }
            this.messageTimestamps[messageId] = now;
        }
        if (prepend) {
            this.queue.prepend(message);
        } else {
            this.queue.append(message);
        }

        this.onEnqueue?.(message);

        // 惰性检查逻辑
        this.enqueueCounter += 1;
        if (this.enqueueCounter % this.checkInterval === 0) {
            this.enqueueCounter = 0;
            const currentLen = this.queue.size;
            if (currentLen > this.maxLen) {
                this.logger.warn(`Message queue length exceeds ${this.maxLen}.`);
                while (this.queue.size > this.maxLen) {
                    this.queue.removeLast();
                }
            }
        }

        return this;
    }

    /**
     * Get the first message in the queue.
     */
    dequeue(): T | undefined {
        const message = this.queue.removeFirst();
        if (message) {
            this.onDequeue?.(message);
        }
        return message;
    }

    // 插入到指定位置
    insert(message: T, index: number) {
        this.queue.insert(message, index);
        return this;
    }

    /**
     * Get the message id.
     * @param message The message to get id from.
     * @private
     */
    private getMessageId(message: T): string {
        try {
            return JSON.stringify(message);
        } catch (e) {
            return message.toString();
        }
    }

    private cleanup() {
        const { debounceTime = 0 } = this.config;
        // queue visualization
        if (process.env.NODE_ENV === 'development') {
            this.logger.debug(`Message queue length: ${this.queue.size}`);
        }
        const now = Date.now();
        Object.keys(this.messageTimestamps).forEach((id) => {
            if (now - this.messageTimestamps[id] >= debounceTime) {
                delete this.messageTimestamps[id];
            }
        });
    }

    setEventListener(event: 'enqueue' | 'dequeue', callback: (message: T) => void) {
        if (event === 'enqueue') {
            this.onEnqueue = callback;
        } else if (event === 'dequeue') {
            this.onDequeue = callback;
        }
        return this;
    }

    isEmpty() {
        return this.queue.isEmpty;
    }

    get size() {
        return this.queue.size;
    }
}
