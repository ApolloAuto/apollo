import { noop } from '@dreamview/dreamview-carviz/src/utils/similarFunctions';
import { BaseEvent } from './BaseEvent';

export interface IMouseDownInfo {
    timeStamp: number;
    x: number;
    y: number;
    event?: MouseEvent;
}

export type InteractionType = 'click' | 'drag' | 'doubleClick';

export interface ICallbacks {
    handleMouseMove?: (event: MouseEvent, mouseDownInfo: IMouseDownInfo) => void;
    handleMouseMoveNotDragging?: (event: MouseEvent) => void;
    handleMouseDown?: (event: MouseEvent) => void;
    handleMouseEnter?: (event: MouseEvent) => void;
    handleMouseLeave?: (event: MouseEvent) => void;
    handleMouseUp: (event: MouseEvent, interaction: InteractionType) => void;
}

export class MouseInteractionEvent implements BaseEvent {
    // 0: left, 1: middle, 2: right
    private excludeButtons = [1, 2];

    private mouseDownInfo: IMouseDownInfo = { timeStamp: 0, x: 0, y: 0 };

    // 记录最新的一次点击时间戳
    private lastClickTimeStamp = 0;

    // 双击最大间隔时间
    private doubleClickTimeout = 300; // 双击间隔时间

    private isDragging = false;

    private callbacks: ICallbacks = { handleMouseUp: noop };

    constructor(private element: HTMLElement, callbacks: ICallbacks, private context?: any) {
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
        this.mouseDownInfo = { timeStamp: event.timeStamp, x: event.clientX, y: event.clientY, event };
        this.isDragging = true;
        this.callbacks?.handleMouseDown?.(event);
    };

    private handleMouseMove = (event: MouseEvent) => {
        if (this.excludeButtons.includes(event.button)) return;

        // 屏蔽轻微鼠标抖动
        if (Math.abs(event.clientX - this.mouseDownInfo.x) < 5 && Math.abs(event.clientY - this.mouseDownInfo.y) < 5) {
            return;
        }

        if (this.isDragging) {
            this.callbacks?.handleMouseMove?.(event, this.mouseDownInfo);
        } else {
            this.callbacks?.handleMouseMoveNotDragging?.(event);
        }
    };

    private handleMouseUp = (event: MouseEvent) => {
        if (this.excludeButtons.includes(event.button)) return;
        const deltaX = event.clientX - this.mouseDownInfo.x;
        const deltaY = event.clientY - this.mouseDownInfo.y;
        const deltaT = event.timeStamp - this.mouseDownInfo.timeStamp;

        if (this.isDragging) {
            this.isDragging = false;
            if (deltaT < 200 && Math.sqrt(deltaX * deltaX + deltaY * deltaY) < 5) {
                if (event.timeStamp - this.lastClickTimeStamp <= this.doubleClickTimeout) {
                    this.callbacks.handleMouseUp(event, 'doubleClick');
                } else {
                    this.callbacks.handleMouseUp(event, 'click');
                }
                this.lastClickTimeStamp = event.timeStamp;
            } else {
                this.callbacks.handleMouseUp(event, 'drag');
            }
        }
    };

    destroy(): void {
        this.element.removeEventListener('mousedown', this.handleMouseDown);
        this.element.removeEventListener('mousemove', this.handleMouseMove);
        this.element.removeEventListener('mouseup', this.handleMouseUp);
        this.element.removeEventListener('mouseenter', this.handleMouseEnter);
        this.element.removeEventListener('mouseleave', this.handleMouseLeave);
    }
}
