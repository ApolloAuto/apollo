import * as THREE from 'three';

type SyncFactory<T> = () => T;
type AsyncFactory<T> = () => Promise<T>;

export default class ThreeObjectPool<T extends THREE.Object3D> {
    private pool: T[] = [];

    private readonly syncFactory?: SyncFactory<T>;

    private readonly asyncFactory?: AsyncFactory<T>;

    private readonly maxSize: number;

    private readonly initialize?: (object: T) => void;

    private readonly dispose?: (object: T) => void;

    private readonly reset?: (object: T) => void;

    private readonly type?: string;

    constructor(config: {
        type?: string;
        // 创建工厂函数
        syncFactory?: SyncFactory<T> | null;
        asyncFactory?: AsyncFactory<T> | null;
        maxSize?: number;
        // 初始化
        initialize?: (object: T) => void;
        // 重置
        reset?: (object: T) => void;
        // 销毁
        dispose?: (object: T) => void;
    }) {
        this.syncFactory = config.syncFactory;
        this.asyncFactory = config.asyncFactory;
        this.maxSize = config.maxSize || Infinity;
        this.type = config.type || 'default';
        this.initialize = config.initialize;
        this.dispose = config.dispose;
        this.reset = config.reset;
    }

    acquireSync(): T {
        if (!this.syncFactory) {
            throw new Error('Sync factory is not defined.');
        }

        let object: T;

        if (this.pool.length > 0) {
            object = this.pool.pop()!;
        } else {
            object = this.syncFactory();
            // 执行重置逻辑并绑定类型
            this.initialize?.(object);
            if (object instanceof THREE.Object3D) {
                object.userData.type = this.type;
            }
        }

        if (this.pool.length + 1 > this.maxSize) {
            throw new Error(`${this.type} Object pool reached its maximum size.`);
        }
        this.reset?.(object);
        return object;
    }

    async acquireAsync(): Promise<T> {
        if (!this.asyncFactory) {
            throw new Error('Async factory is not defined.');
        }

        let object: T;

        if (this.pool.length > 0) {
            object = this.pool.pop()!;
        } else {
            object = await this.asyncFactory();
            this.initialize?.(object);
            if (object instanceof THREE.Object3D) {
                object.userData.type = this.type;
            }
        }

        if (this.pool.length + 1 > this.maxSize) {
            throw new Error('Object pool reached its maximum size.');
        }
        this.reset?.(object);
        return object;
    }

    release(object: T): void {
        if (this.pool.length < this.maxSize) {
            this.dispose?.(object);
            this.pool.push(object);
        }
    }

    visualize(): void {}
}
