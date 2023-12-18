import MultiKeyMapNode from './MultiKeyMapNode';
import { Emptyable } from './similarFunctions';

type ObjectType = {
    [key: string]: unknown;
};

export default class MultiKeyMap<K extends ObjectType, T = unknown> {
    root: MultiKeyMapNode<T>;

    size: number;

    constructor() {
        this.root = new MultiKeyMapNode();
        this.size = 0;
    }

    public set(key: K, value: T): void {
        let node = this.root;
        const entries = Object.entries(key).sort();
        entries.forEach(([k, v]) => {
            const childKey = `${k}:${v}`;
            if (!node.children.has(childKey)) {
                node.children.set(childKey, new MultiKeyMapNode());
            }
            node = node.children.get(childKey);
        });

        if (!node.values.has(value)) {
            this.size += 1;
            node.values.add(value);
        }
    }

    public get<TExpected = T>(key: K): TExpected[] {
        let node = this.root;
        const entries = Object.entries(key).sort();
        const allExist = entries.every(([k, v]) => {
            const childKey = `${k}:${v}`;
            if (!node.children.has(childKey)) {
                return false;
            }
            node = node.children.get(childKey);
            return true;
        });
        if (!allExist) {
            return [];
        }
        return Array.from(node.values) as unknown as TExpected[];
    }

    public getByExactKey<TExpected = T>(key: K): Emptyable<TExpected> {
        let node = this.root;
        const entries = Object.entries(key).sort();
        for (let i = 0; i < entries.length; i += 1) {
            const [k, v] = entries[i];
            const childKey = `${k}:${v}`;
            const childNode = node.children.get(childKey);
            if (childNode) {
                node = childNode;
            } else {
                return undefined;
            }
        }
        if (node.children.size > entries.length) {
            return undefined;
        }
        return node.values.values().next().value;
    }

    public delete(key: K, callback?: (value: T) => void): boolean {
        let node = this.root;
        const entries = Object.entries(key).sort();
        const allExist = entries.every(([k, v]) => {
            const childKey = `${k}:${v}`;
            if (!node.children.has(childKey)) {
                return false;
            }
            node = node.children.get(childKey);
            return true;
        });
        if (!allExist) {
            return false;
        }
        node.values.forEach((value) => callback && callback(value));
        this.size -= node.values.size;
        node.values.clear();
        return true;
    }

    public deleteByExactKey(key: K, callback?: (value: T) => void): boolean {
        let node = this.root;
        const entries = Object.entries(key).sort();
        for (let i = 0; i < entries.length; i += 1) {
            const [k, v] = entries[i];
            const childKey = `${k}:${v}`;
            const childNode = node.children.get(childKey);
            if (childNode) {
                node = childNode;
            } else {
                return false;
            }
        }
        if (node.children.size > 0) {
            return false;
        }
        node.values.forEach((value) => callback && callback(value));
        this.size -= node.values.size;
        node.values.clear();
        return true;
    }

    public count(): number {
        return this.size;
    }

    public getAllEntries(): [K, T][] {
        const entries: [K, T][] = [];
        this.traverse((key, value) => {
            entries.push([key, value]);
        });
        return entries;
    }

    public countIf(predicate: (key: K, value: T) => boolean): number {
        let count = 0;
        this.traverse((key, value) => {
            if (predicate(key, value)) {
                count += 1;
            }
        });
        return count;
    }

    private traverse(
        callback: (key: K, value: T) => void,
        node: MultiKeyMapNode<T> = this.root,
        key: K = {} as K,
    ): void {
        Array.from(node.children.entries()).forEach(([childKey, childNode]) => {
            const [k, v] = childKey.split(':');
            const newKey = { ...key, [k]: v };
            childNode.values.forEach((value) => callback(newKey, value));
            this.traverse(callback, childNode, newKey);
        });
    }

    public clear(): void {
        this.root = new MultiKeyMapNode();
        this.size = 0;
    }
}
