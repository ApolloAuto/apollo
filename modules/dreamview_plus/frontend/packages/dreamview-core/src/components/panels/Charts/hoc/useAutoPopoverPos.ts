import { useEffect, useRef } from 'react';
import { usePanelContext } from '@dreamview/dreamview-core/src/components/panels/base/store/PanelStore';

// 获取一个元素距离HTML左边的距离
export function getClientLeft(elem: Element) {
    const selfElem = elem as HTMLElement;
    let left = 0;
    const HTML_TAG_NAME = 'HTML';
    let current = selfElem;
    let target = selfElem;
    while (true) {
        const parentElement = current.parentElement;
        const stylePosition = getComputedStyle(parentElement, null).getPropertyValue('position');
        const isParentIsHTML = parentElement.tagName === HTML_TAG_NAME;
        if (['relative', 'absolute', 'fixed'].includes(stylePosition) || isParentIsHTML) {
            left += target.offsetLeft;
            target = parentElement;
        }
        current = parentElement;
        if (isParentIsHTML) {
            break;
        }
    }

    return left;
}

export function useAutoPopoverPos(className: string, isPopoverOpen: boolean) {
    const { onPanelResize } = usePanelContext();
    const widthRef = useRef<any>();
    const changeClassName = (action: 'add' | 'remove') => {
        const popover = document.getElementsByClassName('js-chart-popover')[0];
        if (!popover) {
            return;
        }
        const timer = setTimeout(() => {
            popover.classList[action]?.(className);
            clearTimeout(timer);
        }, 500);
    };
    const resizeHandler = (width: number) => {
        const elem = document.getElementsByClassName('js-chart-container')[0];
        const popoverWidth = 390;
        widthRef.current = width;
        const leftOffset = getClientLeft(elem);
        const rightOffset = document.documentElement.clientWidth - leftOffset - width;

        if (leftOffset > popoverWidth) {
            changeClassName('remove');
        } else if (rightOffset > popoverWidth) {
            changeClassName('remove');
        } else {
            changeClassName('add');
        }
    };

    // 自动调整popover位置
    useEffect(() => {
        onPanelResize(resizeHandler);
    }, []);

    useEffect(() => {
        if (widthRef.current && isPopoverOpen) {
            resizeHandler(widthRef.current);
        }
    }, [isPopoverOpen]);
}
