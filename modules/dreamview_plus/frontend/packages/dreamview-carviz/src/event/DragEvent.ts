import { BaseEvent } from './BaseEvent';

export class DragEvent implements BaseEvent {
    private isDragging = false;

    constructor(
        private element: HTMLElement,
        private callback: {
            handleMouseMove: (event: MouseEvent) => void;
            handleMouseDown: (event: MouseEvent) => void;
            handleMouseUp: (event: MouseEvent) => void;
        },
    ) {
        this.init();
    }

    init() {
        this.element.addEventListener('mousedown', this.handleMouseDown);
        this.element.addEventListener('mousemove', this.handleMouseMove);
        this.element.addEventListener('mouseup', this.handleMouseUp);
    }

    private handleMouseDown = (event: MouseEvent) => {
        this.isDragging = true;
        this.callback.handleMouseDown(event);
    };

    private handleMouseMove = (event: MouseEvent) => {
        if (this.isDragging) {
            this.callback.handleMouseMove(event);
        }
    };

    private handleMouseUp = (event: MouseEvent) => {
        if (this.isDragging) {
            this.isDragging = false;
            this.callback.handleMouseUp(event);
        }
    };

    destroy() {
        this.element.removeEventListener('mousedown', this.handleMouseDown);
        this.element.removeEventListener('mousemove', this.handleMouseMove);
        this.element.removeEventListener('mouseup', this.handleMouseUp);
    }
}
