import { BaseEvent } from './BaseEvent';

export class ClickEvent implements BaseEvent {
    private mouseDownInfo = { timeStamp: 0, x: 0, y: 0 };

    constructor(private element: HTMLElement, private readonly callback: (event: MouseEvent) => void) {
        this.element = element;
        this.callback = callback;
        this.init();
    }

    init() {
        this.element.addEventListener('mousedown', this.handleMouseDown);
        this.element.addEventListener('mouseup', this.handleMouseUp);
    }

    destroy() {
        this.element.removeEventListener('mousedown', this.handleMouseDown);
        this.element.removeEventListener('mouseup', this.handleMouseUp);
    }

    private handleMouseDown = (event: MouseEvent) => {
        this.mouseDownInfo = { timeStamp: event.timeStamp, x: event.clientX, y: event.clientY };
    };

    private handleMouseUp = (event: MouseEvent) => {
        const deltaX = event.clientX - this.mouseDownInfo.x;
        const deltaY = event.clientY - this.mouseDownInfo.y;
        const deltaT = event.timeStamp - this.mouseDownInfo.timeStamp;

        if (deltaT < 200 && Math.sqrt(deltaX * deltaX + deltaY * deltaY) < 5) {
            this.callback(event);
        }
    };
}
