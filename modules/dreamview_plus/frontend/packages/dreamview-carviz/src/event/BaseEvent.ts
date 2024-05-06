export abstract class BaseEvent {
    // 初始化事件
    abstract init(): void;

    // 销毁事件
    abstract destroy(): void;
}
