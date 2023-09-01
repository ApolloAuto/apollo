import React, { useEffect, useRef } from 'react';
import { Observable, filter, fromEvent } from 'rxjs';
import { FunctionalKey } from '../../../../store/EventHandlersStore';

export type KeyHandlers = {
    keys: string[];
    functionalKey?: FunctionalKey;
    handler: (event: KeyboardEvent) => void | boolean | undefined;
};

type KeyListenerProps = {
    active?: boolean;
    gloabl?: boolean;
    keyDownHandlers?: KeyHandlers[];
    keyUpHandlers?: KeyHandlers[];
};

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
                });
        });
    };
}

export default function KeyListener(props: KeyListenerProps) {
    const { active = false, keyDownHandlers, keyUpHandlers } = props;
    const elementRef = useRef<HTMLDivElement>();
    const activeRef = useRef<boolean>(active);

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

        if (keyUpHandlers) {
            const len = keyUpHandlers.length;
            for (let i = 0; i < len; i += 1) {
                const keyUpEvent = fromEvent(document, 'keyup');
                const setKeyUp = getMultiPressedKey(keyUpEvent);
                const keyUpHandler = keyUpHandlers[i];
                setKeyUp(
                    (event) => {
                        if (activeRef.current) {
                            keyUpHandler?.handler(event as KeyboardEvent);
                        }
                    },
                    keyUpHandler?.keys,
                    keyUpHandler?.functionalKey,
                );
            }
        }
    }, [keyDownHandlers, keyUpHandlers]);

    return (
        <div
            ref={elementRef}
            style={{
                display: 'none',
            }}
        />
    );
}
