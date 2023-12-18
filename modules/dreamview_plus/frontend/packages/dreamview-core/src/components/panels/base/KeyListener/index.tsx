/* eslint-disable no-restricted-syntax */
/* eslint-disable react-hooks/exhaustive-deps */
import React, { useCallback, useEffect, useRef } from 'react';
import { Observable, Subscription, filter, fromEvent } from 'rxjs';
import { FunctionalKey } from '../../../../store/EventHandlersStore';

export type KeyHandlers = {
    keys: string[];
    functionalKey?: FunctionalKey;
    handler: (event: KeyboardEvent) => void | boolean | undefined;
    discriptor?: React.ReactNode;
    isGloable?: boolean;
};

type KeyListenerProps = {
    active?: boolean;
    gloabl?: boolean;
    keyDownHandlers?: KeyHandlers[];
    keyUpHandlers?: KeyHandlers[];
};

export default function KeyListener(props: KeyListenerProps) {
    const { active = false, keyDownHandlers, keyUpHandlers } = props;
    const elementRef = useRef<HTMLDivElement>();
    const activeRef = useRef<boolean>(active);
    const subscriptionsRef = useRef<Subscription[]>([]);

    const getMultiPressedKey = useCallback(
        (observableEvent: Observable<Event>) =>
            (cb: (event: Event) => void, keys: string[], functionalKey?: FunctionalKey) => {
                let isPressedKeys = new Array<boolean>(keys.length).fill(false);
                keys.forEach((key, index) => {
                    subscriptionsRef.current.push(
                        observableEvent
                            .pipe(
                                filter((event) => {
                                    if (event instanceof KeyboardEvent) {
                                        const lowerKey = key.toLowerCase();
                                        const eventKey = event.key?.toLowerCase();

                                        const clearerTimer = setTimeout(() => {
                                            isPressedKeys[index] = false;
                                            clearTimeout(clearerTimer);
                                        }, 200);

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
                            }),
                    );
                });
            },
        [],
    );

    useEffect(() => {
        activeRef.current = active;
    }, [active]);

    useEffect(() => {
        if (keyDownHandlers) {
            const len = keyDownHandlers.length;
            for (let i = 0; i < len; i += 1) {
                const keyDownEvent = fromEvent(document, 'keydown');
                const setKeyDown = getMultiPressedKey(keyDownEvent);
                const keyDownHandler = keyDownHandlers[i];
                setKeyDown(
                    (event) => {
                        if (activeRef.current) {
                            keyDownHandler?.handler(event as KeyboardEvent);
                        }
                    },
                    keyDownHandler?.keys,
                    keyDownHandler?.functionalKey,
                );
            }
        }

        return () => {
            // 想要清空keyDownEventRef.current注册的所有回调
            for (const sp of subscriptionsRef.current) {
                sp.unsubscribe();
            }

            subscriptionsRef.current = [];
        };
    }, [keyDownHandlers]);

    return (
        <div
            ref={elementRef}
            style={{
                display: 'none',
            }}
        />
    );
}
