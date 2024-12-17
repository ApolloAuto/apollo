import { StoreManager } from './StoreManager';

const DB_NAME = 'Dreamview_Plus';
// eslint-disable-next-line @typescript-eslint/no-var-requires
const packageVersion = require('@dreamview/dreamview-core/package.json').version;

// 通过版本号生成数据库版本号，考虑到版本号可能有多个小数点，需要所有版本号都转为整数
const majorVersion = packageVersion
    .split('.')
    .reduce((acc: number, cur: string, index: number) => acc * 10 ** index + parseInt(cur, 10), 0);

/**
 *  IndexedDBStorage types for each store
 */
export interface StoreTypes {
    DreamviewPlus: any;
    ProtoCache: string;
    ProtoDescriptor: any;
}

export type StoreNames = keyof StoreTypes;

export class IndexedDBStorage {
    private db: IDBDatabase | null = null;

    private storeManagers: Map<StoreNames, StoreManager<StoreTypes[StoreNames]>> = new Map();

    constructor(private stores: StoreNames[]) {
        this.initializeDatabase().catch((error) => console.error('Database initialization error:', error));
    }

    private async initializeDatabase(): Promise<void> {
        this.db = await this.openDatabase();
    }

    private async openDatabase(): Promise<IDBDatabase> {
        return new Promise((resolve, reject) => {
            const openDB = () => {
                const request = indexedDB.open(`${DB_NAME}`, majorVersion || 1);
                request.onerror = (event) => {
                    const error = (event.target as IDBOpenDBRequest).error;
                    // 如果是版本错误，表示版本降级
                    if (error?.name === 'VersionError') {
                        // 删除旧数据库
                        const deleteRequest = indexedDB.deleteDatabase(`${DB_NAME}`);
                        deleteRequest.onsuccess = () => {
                            // 重新打开数据库
                            openDB();
                        };
                        deleteRequest.onerror = () => {
                            reject(error);
                        };
                        deleteRequest.onblocked = () => {
                            reject(new Error('Database deletion blocked'));
                        };
                    } else {
                        reject(error);
                    }
                };
                request.onupgradeneeded = (event) => {
                    const db = (event.target as IDBOpenDBRequest).result;
                    this.stores.forEach((storeName) => {
                        if (!db.objectStoreNames.contains(storeName)) {
                            db.createObjectStore(storeName, { keyPath: 'key' });
                        }
                    });
                };
                request.onsuccess = (event) => {
                    resolve((event.target as IDBOpenDBRequest).result);
                };
            };
            openDB();
        });
    }

    async getStoreManager<T>(storeName: StoreNames): Promise<StoreManager<T>> {
        if (!this.db) {
            await this.initializeDatabase();
        }

        let manager = this.storeManagers.get(storeName);

        if (!manager) {
            if (this.db) {
                manager = new StoreManager<StoreTypes[StoreNames]>(this.db, storeName);
                this.storeManagers.set(storeName, manager);
            } else {
                throw new Error(`Database not initialized or Store ${storeName} is not valid`);
            }
        }

        return manager as StoreManager<T>;
    }
}

export const indexedDBStorage = new IndexedDBStorage(['DreamviewPlus', 'ProtoDescriptor', 'ProtoCache']);
