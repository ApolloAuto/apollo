import { BaseEvent } from './BaseEvent';

export interface ICallbacks {
    handleMouseEnter?: (event: MouseEvent) => void;
    handleMouseLeave?: (event: MouseEvent) => void;
    handleMouseMove?: (event: MouseEvent) => void;
}

export class MouseOverEvent implements BaseEvent {
    private callbacks: ICallbacks = {};

    constructor(private element: HTMLElement, callbacks: ICallbacks) {
        this.callbacks = callbacks;
        this.init();
    }

    init(): void {
        this.element.addEventListener('mouseenter', this.handleMouseEnter);
        this.element.addEventListener('mouseleave', this.handleMouseLeave);
        this.element.addEventListener('mousemove', this.handleMouseMove);
    }

    private handleMouseEnter = (event: MouseEvent) => {
        this.callbacks?.handleMouseEnter?.(event);
    };

    private handleMouseLeave = (event: MouseEvent) => {
        this.callbacks?.handleMouseLeave?.(event);
    };

    private handleMouseMove = (event: MouseEvent) => {
        this.callbacks?.handleMouseMove?.(event);
    };

    destroy(): void {
        this.element.removeEventListener('mouseenter', this.handleMouseEnter);
        this.element.removeEventListener('mouseleave', this.handleMouseLeave);
        this.element.removeEventListener('mousemove', this.handleMouseMove);
    }
}
