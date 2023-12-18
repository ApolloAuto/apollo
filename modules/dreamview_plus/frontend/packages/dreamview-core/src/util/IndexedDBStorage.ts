const DB_NAME = 'Dreamview_Plus';
// eslint-disable-next-line @typescript-eslint/no-var-requires
const packageVersion = require('../../package.json').version;

// 提取主版本号
const majorVersion = parseInt(packageVersion.split('.')[0], 10);

// 检查是否为有效的正整数
if (!Number.isInteger(majorVersion) || majorVersion <= 0) {
    throw new Error('Invalid database version number');
}

const DEFAULT_STORE_NAME = 'Dreamview_Plus_Store';

interface StorageItem<T> {
    key: string;
    value: T;
    time: number;
    timeout?: number;
}

export class IndexedDBStorage<T = any> {
    private static readonly DB_NAME: string = `${DB_NAME}_${majorVersion || 1}`;

    private readonly storeName: string;

    private static readonly DB_VERSION: number = majorVersion || 1;

    constructor(storeName?: string) {
        this.storeName = storeName || DEFAULT_STORE_NAME;
    }

    private async openDatabase(): Promise<IDBDatabase> {
        return new Promise((resolve, reject) => {
            const request = indexedDB.open(IndexedDBStorage.DB_NAME, IndexedDBStorage.DB_VERSION);

            request.onerror = (event) => {
                const error = (event.target as IDBOpenDBRequest).error;
                console.error('Database error:', error);
                reject(error);
            };

            request.onupgradeneeded = (event) => {
                const db = (event.target as IDBOpenDBRequest).result;
                if (!db.objectStoreNames.contains(this.storeName)) {
                    db.createObjectStore(this.storeName, { keyPath: 'key' });
                }
            };

            request.onsuccess = (event) => {
                const db = (event.target as IDBOpenDBRequest).result;
                resolve(db);
            };
        });
    }

    private async getStore(mode: IDBTransactionMode): Promise<IDBObjectStore> {
        const db = await this.openDatabase();
        const tx = db.transaction(this.storeName, mode);
        return tx.objectStore(this.storeName);
    }

    async setItem(key: string, value: T, timeout?: number): Promise<void> {
        const store = await this.getStore('readwrite');
        return new Promise((resolve, reject) => {
            const request = store.put({ key, value, time: Date.now(), timeout });
            request.onsuccess = () => resolve();
            request.onerror = () => {
                const error = (request as IDBRequest).error;
                reject(error);
            };
        });
    }

    async getItem(key: string): Promise<T | null> {
        const store = await this.getStore('readonly');
        return new Promise((resolve, reject) => {
            const request = store.get(key);
            request.onsuccess = () => {
                const result = (request as IDBRequest).result as StorageItem<T> | undefined;
                if (result && (!result.timeout || Date.now() - result.time < result.timeout)) {
                    resolve(result.value);
                } else {
                    resolve(null);
                }
            };
            request.onerror = () => {
                const error = (request as IDBRequest).error;
                reject(error);
            };
        });
    }

    async removeItem(key: string): Promise<void> {
        const store = await this.getStore('readwrite');
        return new Promise((resolve, reject) => {
            const request = store.delete(key);
            request.onsuccess = () => resolve();
            request.onerror = () => {
                const error = (request as IDBRequest).error;
                reject(error);
            };
        });
    }

    async removeItemsByNamespace(namespace: string): Promise<void> {
        const store = await this.getStore('readwrite');
        return new Promise((resolve, reject) => {
            const request = store.openCursor();
            request.onsuccess = (event) => {
                const cursor = (event.target as IDBRequest).result as IDBCursorWithValue | null;
                if (cursor) {
                    if (cursor.key.toString().startsWith(namespace)) {
                        cursor.delete();
                    }
                    cursor.continue();
                } else {
                    resolve();
                }
            };
            request.onerror = () => {
                const error = (request as IDBRequest).error;
                reject(error);
            };
        });
    }
}
