import { noop } from '@dreamview/dreamview-carviz/src/utils/similarFunctions';
import { BaseEvent } from './BaseEvent';

export interface IMouseDownInfo {
    timeStamp: number;
    x: number;
    y: number;
    event?: MouseEvent;
}

export type ContinuousMouseInteractionType = 'click' | 'rightClick' | 'doubleClick';

export interface ICallbacks {
    handleMouseMove?: (event: MouseEvent, mouseDownInfo: IMouseDownInfo) => void;
    handleMouseMoveNotDragging?: (event: MouseEvent) => void;
    handleMouseDown?: (event: MouseEvent, interaction: ContinuousMouseInteractionType) => void;
    handleMouseUp: (event: MouseEvent, interaction: ContinuousMouseInteractionType) => void;
    handleMouseEnter?: (event: MouseEvent) => void;
    handleMouseLeave?: (event: MouseEvent) => void;
}

export class ContinuousMouseEvent implements BaseEvent {
    // 0: left, 1: middle, 2: right
    private excludeButtons = [1];

    private leftMouseDownInfo: IMouseDownInfo = { timeStamp: 0, x: 0, y: 0 };

    private rightMouseDownInfo: IMouseDownInfo = { timeStamp: 0, x: 0, y: 0 };

    // 记录最新的一次点击时间戳
    private lastClickTimeStamp = 0;

    // 双击最大间隔时间
    private doubleClickTimeout = 300; // 双击间隔时间

    // 是否开启交互（鼠标滑动持续触发）
    private isInteracting = false;

    private callbacks: ICallbacks = { handleMouseUp: noop };

    constructor(private element: HTMLElement, callbacks: ICallbacks, context?: any) {
        this.callbacks = {
            handleMouseUp: callbacks.handleMouseUp.bind(context),
            handleMouseMove: callbacks.handleMouseMove?.bind(context),
            handleMouseMoveNotDragging: callbacks.handleMouseMoveNotDragging?.bind(context),
            handleMouseDown: callbacks.handleMouseDown?.bind(context),
            handleMouseEnter: callbacks.handleMouseEnter?.bind(context),
            handleMouseLeave: callbacks.handleMouseLeave?.bind(context),
        };
        this.init();
    }

    init(): void {
        this.element.addEventListener('mousedown', this.handleMouseDown);
        this.element.addEventListener('mousemove', this.handleMouseMove);
        this.element.addEventListener('mouseup', this.handleMouseUp);
        this.element.addEventListener('mouseenter', this.handleMouseEnter);
        this.element.addEventListener('mouseleave', this.handleMouseLeave);
    }

    private handleMouseEnter = (event: MouseEvent) => {
        this.callbacks?.handleMouseEnter?.(event);
    };

    private handleMouseLeave = (event: MouseEvent) => {
        this.callbacks?.handleMouseLeave?.(event);
    };

    private handleMouseDown = (event: MouseEvent) => {
        if (this.excludeButtons.includes(event.button)) return;
        if (event.button === 2) {
            this.rightMouseDownInfo = { timeStamp: event.timeStamp, x: event.clientX, y: event.clientY, event };
        } else if (event.button === 0) {
            this.leftMouseDownInfo = { timeStamp: event.timeStamp, x: event.clientX, y: event.clientY, event };
            this.isInteracting = true;
        }
        this.callbacks?.handleMouseDown?.(event, event.button === 2 ? 'rightClick' : 'click');
    };

    private handleMouseMove = (event: MouseEvent) => {
        // 屏蔽轻微鼠标抖动
        if (
            Math.abs(event.clientX - this.leftMouseDownInfo.x) < 5 &&
            Math.abs(event.clientY - this.leftMouseDownInfo.y) < 5
        ) {
            return;
        }

        if (this.isInteracting) {
            this.callbacks?.handleMouseMove(event, this.leftMouseDownInfo);
        } else {
            this.callbacks?.handleMouseMoveNotDragging?.(event);
        }
    };

    private handleMouseUp = (event: MouseEvent) => {
        if (this.excludeButtons.includes(event.button)) return;

        if (event.button === 0) {
            const deltaX = event.clientX - this.leftMouseDownInfo.x;
            const deltaY = event.clientY - this.leftMouseDownInfo.y;
            const deltaT = event.timeStamp - this.leftMouseDownInfo.timeStamp;

            if (this.isInteracting) {
                if (deltaT < 200 && Math.sqrt(deltaX * deltaX + deltaY * deltaY) < 5) {
                    if (event.timeStamp - this.lastClickTimeStamp <= this.doubleClickTimeout) {
                        this.isInteracting = false;
                        this.callbacks?.handleMouseUp(event, 'doubleClick');
                    } else {
                        this.callbacks.handleMouseUp(event, 'click');
                    }
                    this.lastClickTimeStamp = event.timeStamp;
                }
            }
        }

        if (event.button === 2) {
            const deltaX = event.clientX - this.rightMouseDownInfo.x;
            const deltaY = event.clientY - this.rightMouseDownInfo.y;
            const deltaT = event.timeStamp - this.rightMouseDownInfo.timeStamp;

            if (this.isInteracting) {
                if (deltaT < 200 && Math.sqrt(deltaX * deltaX + deltaY * deltaY) < 5) {
                    this.isInteracting = false;
                    this.callbacks.handleMouseUp(event, 'rightClick');
                }
            }
        }
    };

    destroy(): void {
        this.element.removeEventListener('mousedown', this.handleMouseDown);
        this.element.removeEventListener('mousemove', this.handleMouseMove);
        this.element.removeEventListener('mouseup', this.handleMouseUp);
    }
}
