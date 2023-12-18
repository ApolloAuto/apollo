import React, { useMemo, useRef, useCallback, useEffect } from 'react';

import once from 'lodash/once';

interface EventCollectionType {
    elem: any;
    cb: (flag: boolean) => void;
}

type IRegisitScrollEvent = (uid: string, elem: any, cb: (flag: boolean) => void) => () => any;

// 获取一个元素距离HTML左边的距离
export function getOffsetTop(elem: Element, rootElem?: any) {
    const selfElem = elem as HTMLElement;
    let top = 0;
    const ROOT_TAG_NAME = 'HTML';
    let current = selfElem;
    let target = selfElem;
    while (true) {
        const parentElement = current.parentElement;
        const stylePosition = getComputedStyle(parentElement, null).getPropertyValue('position');
        const hasFindRoot = parentElement.tagName === ROOT_TAG_NAME || parentElement === rootElem;
        if (['relative', 'absolute', 'fixed'].includes(stylePosition) || hasFindRoot) {
            top += target.offsetTop;
            target = parentElement;
        }

        current = parentElement;
        if (hasFindRoot) {
            break;
        }
    }

    return top;
}

export const viewBoxPosInfoContext = React.createContext<{
    regisitScrollEvent: IRegisitScrollEvent;
}>(null);

export function useViewBoxPosInfo() {
    return React.useContext(viewBoxPosInfoContext);
}

export function useCreateViewBoxPosInfo() {
    const scrollEventCollection = useRef<Record<string, EventCollectionType>>({});
    const info = useRef({
        scrollElem: null,
    });
    const resizeobServer = useRef<any>();

    const changeHandler = () => {
        const scrollElem = info.current.scrollElem;
        if (!scrollElem) {
            return;
        }
        const max = scrollElem.scrollTop + scrollElem.offsetHeight;
        const min = scrollElem.scrollTop;
        Object.keys(scrollEventCollection.current).forEach((uid) => {
            const { elem: childElem, cb } = scrollEventCollection.current[uid];
            if (childElem) {
                const top = getOffsetTop(childElem, scrollElem);
                const height = childElem.offsetHeight;
                const isInView = height + top > min && top < max;
                cb(isInView);
            } else {
                cb(false);
            }
        });
    };

    const bindEventOnce = useCallback(
        once((elem: any) => {
            info.current.scrollElem = elem;
            elem.addEventListener('scroll', () => {
                changeHandler();
            });

            resizeobServer.current = new ResizeObserver(() => {
                changeHandler();
            });
            resizeobServer.current.observe(elem);
        }),
        [],
    );

    const onRef = useCallback((elem: any) => {
        if (elem) {
            bindEventOnce(elem);
        }
    }, []);

    const regisitScrollEvent = useCallback((uid: string, elem: any, cb: (isInView: boolean) => void) => {
        scrollEventCollection.current[uid] = { elem, cb };
        changeHandler();
        return () => {
            delete scrollEventCollection.current[uid];
            changeHandler();
        };
    }, []);

    useEffect(
        () => () => {
            resizeobServer.current.disconnect();
        },
        [],
    );

    return useMemo(
        () => ({
            onRef,
            contextValue: {
                regisitScrollEvent,
            },
        }),
        [onRef, regisitScrollEvent],
    );
}
