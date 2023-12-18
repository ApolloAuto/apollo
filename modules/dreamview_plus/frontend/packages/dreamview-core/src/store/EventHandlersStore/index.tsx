import React, { createContext, useCallback, useContext, useMemo, useRef, useState } from 'react';
import { fromEvent, filter, Observable, Subject, OperatorFunction, ReplaySubject } from 'rxjs';
import EventEmitter from 'eventemitter3';

export * from './eventType';

export type FunctionalKey = 'ctrlKey' | 'metaKey' | 'shiftKey';

type DragEventName = 'drag' | 'dragstart' | 'dragend' | 'dragover' | 'dragenter' | 'dragleave' | 'drop';

export type CustomizeEvent = {
    publish: (value: unknown) => void;
    subscribe: (cb: (value: unknown) => void) => void;
    removeSubscribe: () => void;
    clearSubscribe: () => void;
    publishOnce: (value: unknown) => void;
};

/**
 * @observableEvent 受观测的Event对象
 * @setFilterKey 选择要监听的按键名称，需使用useEffect包起来
 */
type EventHandlers = Record<
    string,
    {
        observableEvent?: Observable<Event>;
        setFilterKey?: (cb: (event: Event) => void, key: string, functionalKey?: FunctionalKey) => any;
        getSubscribedEvent?: (cb: (event: Event) => void) => any;
        setMultiPressedKey?: (cb: (event: Event) => void, keys: string[], functionalKey?: FunctionalKey) => any;
    }
> & {
    customizeSubs?: {
        // subscriber: React.MutableRefObject<Subscriber<any>>;
        reigisterCustomizeEvent: (eventName: string) => void;
        getCustomizeEvent: (eventName: string) => CustomizeEvent;
    };
    dragEvent?: {
        registerDragEvent: (
            target: HTMLElement,
            dragEventName: DragEventName,
            cb: (event: DragEvent) => void,
            ...operations: OperatorFunction<any, any>[]
        ) => Observable<Event>;
    };
} & {
    EE: EventEmitter;
};

export const EventHandlersContext = createContext<EventHandlers | undefined>(undefined);

type ProviderProps = {
    children?: React.ReactNode;
};

/**
 * @example
 * const eventHandlers = useEventHandlersContext();
 * const { setFilterKey, setMultiPressedKey } = eventHandlers.keydown;
 * setMultiPressedKey(
 *     (event) => {
 *          if (event instanceof KeyboardEvent) {
 *              console.log('invoked');
 *          }
 *      },
 *      ['a', 'x'],
 *      'ctrlKey',
 *  );
 *
 * 自定义发布订阅：
 * customizeSubs.reigisterCustomizeEvent('test');
 * customEventRef.current = customizeSubs.getCustomizeEvent('test');
 * setTimeout(() => {
 *     customEventRef.current.subscribe((val) => {
 *          console.log('val', val);
 *     });
 * }, 5000);
 *
 * <Button
 *       onClick={() => {
 *           customEventRef.current.publish(Math.random() * 100);
 *       }}
 *   >
 *       publish
 * </Button>
 */
export function EventHandlersProvider({ children }: ProviderProps): React.ReactElement {
    const [EE] = useState(() => new EventEmitter());
    const customizeEventMapRef = useRef<Map<string, CustomizeEvent>>(new Map());
    const publishEventQueueMap = useRef<Map<string, unknown[]>>(new Map());
    const triggerCleanCache = (subject: Subject<unknown>, publishEventQueue: unknown[]) => {
        let val;
        // eslint-disable-next-line no-cond-assign
        while ((val = publishEventQueue.shift())) {
            subject.next(val);
        }
    };
    const reigisterCustomizeEvent = useCallback((eventName: string) => {
        if (!customizeEventMapRef.current.has(eventName)) {
            const subject = new ReplaySubject<unknown>();
            publishEventQueueMap.current.set(eventName, []);

            const publish = (value: unknown) => {
                const publishEventQueue = publishEventQueueMap.current.get(eventName);
                if (subject.observed) {
                    // 已经有订阅者了
                    triggerCleanCache(subject, publishEventQueue);
                    subject.next(value);
                } else {
                    // 还没有订阅者
                    publishEventQueue.push(value);
                }
            };

            const subscribe = (cb: (value: unknown) => void) => {
                subject.subscribe(cb);
                const publishEventQueue = publishEventQueueMap.current.get(eventName);
                if (publishEventQueue.length > 0) {
                    triggerCleanCache(subject, publishEventQueue);
                }
            };

            const removeSubscribe = () => {
                subject.unsubscribe();
                customizeEventMapRef.current.delete(eventName);
            };

            const clearSubscribe = () => {
                if (subject.observed) {
                    subject.unsubscribe();
                }
            };

            const publishOnce = (value: unknown) => {
                publish(value);
                setTimeout(() => {
                    removeSubscribe();
                }, 0);
            };

            customizeEventMapRef.current.set(eventName, {
                publish,
                subscribe,
                removeSubscribe,
                publishOnce,
                clearSubscribe,
            });
        }
    }, []);
    const getCustomizeEvent = (eventName: string): CustomizeEvent => customizeEventMapRef.current.get(eventName);
    const keyDownEvent = useMemo(() => fromEvent(document, 'keydown'), []);
    const keyUpEvent = useMemo(() => fromEvent(document, 'keyup'), []);
    const clickEvent = useMemo(() => fromEvent(document, 'click'), []);
    const mouseoverEvent = useMemo(() => fromEvent(document, 'mouseover'), []);
    const mouseoutEvent = useMemo(() => fromEvent(document, 'mouseout'), []);
    const scrollEvent = useMemo(() => fromEvent(document, 'scroll'), []);

    function getMultiPressedKey(observableEvent: Observable<Event>) {
        return (cb: (event: Event) => void, keys: string[], functionalKey?: FunctionalKey) => {
            let isPressedKeys = new Array<boolean>(keys.length).fill(false);
            keys.forEach((key, index) => {
                observableEvent
                    .pipe(
                        filter((event) => {
                            if (event instanceof KeyboardEvent) {
                                const lowerKey = key.toLowerCase();
                                const eventKey = event.key?.toLowerCase();

                                if (functionalKey) {
                                    return event[functionalKey] && eventKey === lowerKey;
                                }

                                return eventKey === lowerKey;
                            }

                            return false;
                        }),
                    )
                    .subscribe((event) => {
                        isPressedKeys[index] = true;
                        if (isPressedKeys.reduce((prev, cur) => prev && cur, true)) {
                            cb(event);
                            isPressedKeys = isPressedKeys.fill(false);
                        } else {
                            event.preventDefault();
                        }
                    });
            });
        };
    }

    const usePressedKey = useCallback(
        (cb: (event: Event) => void, key: string, functionalKey?: FunctionalKey) => {
            keyDownEvent
                .pipe(
                    filter((event, _) => {
                        const lowerKey = key.toLowerCase();
                        const eventKey = (event as KeyboardEvent).key?.toLocaleLowerCase();

                        if (functionalKey) {
                            return (event as KeyboardEvent)[functionalKey] && eventKey === lowerKey;
                        }
                        return eventKey === lowerKey;
                    }),
                )
                ?.subscribe(cb);
        },
        [keyDownEvent],
    );

    const useUpKey = useCallback(
        (cb: (event: Event) => void, key: string, functionalKey?: FunctionalKey) => {
            keyUpEvent
                .pipe(
                    filter((event, _) => {
                        const lowerKey = key.toLowerCase();
                        const eventKey = (event as KeyboardEvent).key?.toLocaleLowerCase();

                        if (functionalKey) {
                            return (event as KeyboardEvent)[functionalKey] && eventKey === lowerKey;
                        }
                        return eventKey === lowerKey;
                    }),
                )
                ?.subscribe(cb);
        },
        [keyUpEvent],
    );

    const getSubscribedEvent = (obs: Observable<Event>) => (cb: (event: Event) => void) => {
        obs.subscribe(cb);
    };

    const registerDragEvent = (
        target: HTMLElement,
        dragEventName: DragEventName,
        cb: (event: DragEvent) => void,
        ...operations: OperatorFunction<any, any>[]
    ) => {
        const observableDrag = fromEvent<DragEvent>(target, dragEventName);
        if (operations.length > 0) {
            // eslint-disable-next-line no-restricted-syntax
            for (const operation of operations) {
                observableDrag.pipe(operation).subscribe(cb);
            }
        } else {
            observableDrag.subscribe(cb);
        }

        return observableDrag;
    };

    const store = useMemo(
        () => ({
            EE,
            keydown: {
                observableEvent: keyDownEvent,
                setFilterKey: usePressedKey,
                setMultiPressedKey: getMultiPressedKey(keyDownEvent),
            },
            keyup: {
                observableEvent: keyUpEvent,
                setFilterKey: useUpKey,
                setMultiPressedKey: getMultiPressedKey(keyUpEvent),
            },
            click: {
                observableEvent: clickEvent,
                getSubscribedEvent: getSubscribedEvent(clickEvent),
            },
            mouseover: {
                observableEvent: mouseoverEvent,
                getSubscribedEvent: getSubscribedEvent(mouseoverEvent),
            },
            mouseout: {
                observableEvent: mouseoutEvent,
                getSubscribedEvent: getSubscribedEvent(mouseoutEvent),
            },
            scrollEvent: {
                observableEvent: scrollEvent,
                getSubscribedEvent: getSubscribedEvent(scrollEvent),
            },
            customizeSubs: {
                reigisterCustomizeEvent,
                getCustomizeEvent,
            },
            dragEvent: {
                registerDragEvent,
            },
        }),
        [
            EE,
            clickEvent,
            keyDownEvent,
            keyUpEvent,
            mouseoutEvent,
            mouseoverEvent,
            reigisterCustomizeEvent,
            scrollEvent,
            usePressedKey,
            useUpKey,
        ],
    );

    return <EventHandlersContext.Provider value={store}>{children}</EventHandlersContext.Provider>;
}

export function useEventHandlersContext() {
    return useContext(EventHandlersContext);
}

export function useEventEmitter() {
    return useContext(EventHandlersContext).EE;
}
