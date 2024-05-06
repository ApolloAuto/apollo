export type FunctionType<T = any, E = Event> =
    | ((data?: T, nativeEvent?: E) => void)
    | ((data?: T, nativeEvent?: E) => Promise<void>);

export interface EventListener<T> {
    callback: FunctionType<T>;
    priority: number;
    once: boolean;
}

export interface EventBusEvents {
    [eventType: string]: EventListener<any>[];
}

export interface EventMap {
    [eventType: string]: { data?: any; nativeEvent?: Event };
}

export interface EventListenerOptions<T> {
    priority?: number;
    once?: boolean;
}

function isAsyncFunction(func: FunctionType): boolean {
    return func.constructor.name === 'AsyncFunction';
}

class EventBus<T extends EventMap> {
    private events: EventBusEvents = {};

    public on<K extends keyof T>(
        eventType: K,
        callback: FunctionType<any, any>,
        options: EventListenerOptions<T[K]> = {
            priority: 0,
            once: false,
        },
    ): void {
        if (!this.events[eventType as string]) {
            this.events[eventType as string] = [];
        }

        const { priority = 0, once = false } = options;

        this.events[eventType as string].push({ callback, priority, once });
        this.events[eventType as string].sort((a, b) => b.priority - a.priority);
    }

    public off<K extends keyof T>(eventType: K, callback: FunctionType<any, any>): void {
        if (this.events[eventType as string]) {
            this.events[eventType as string] = this.events[eventType as string].filter(
                (listener) => listener.callback !== callback,
            );
        }
    }

    public async emit<K extends keyof T>(eventType: K, payload?: T[K]): Promise<void> {
        const { data, nativeEvent } = payload ?? {};

        if (this.events[eventType as string]) {
            // eslint-disable-next-line no-restricted-syntax
            for (const listener of [...this.events[eventType as string]]) {
                try {
                    if (isAsyncFunction(listener.callback)) {
                        await listener.callback(data, nativeEvent); // eslint-disable-line no-await-in-loop
                    } else {
                        listener.callback(data, nativeEvent);
                    }

                    if (listener.once) {
                        this.off(eventType, listener.callback);
                    }
                } catch (error) {
                    console.error(`Error in event listener for event "${eventType as string}":`, error);
                }
            }
        }
    }

    public allEventTypes(): string[] {
        return Object.keys(this.events);
    }

    public clear(): void {
        this.events = {};
    }
}

export const eventBus = new EventBus();
