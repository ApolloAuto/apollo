interface StorageItem<T> {
    key: string;
    value: T;
    time: number;
    timeout?: number;
}
export class StoreManager<T = any> {
    constructor(private db: IDBDatabase, private storeName: string) {}

    async setItem(key: string, value: T, timeout?: number): Promise<void> {
        const transaction = this.db.transaction(this.storeName, 'readwrite');
        const store = transaction.objectStore(this.storeName);
        return new Promise((resolve, reject) => {
            const request = store.put({ key, value, time: Date.now(), timeout });
            request.onsuccess = () => resolve();
            request.onerror = () => reject(request.error);
        });
    }

    async getItem(key: string): Promise<T | null> {
        const transaction = this.db.transaction(this.storeName, 'readonly');
        const store = transaction.objectStore(this.storeName);
        return new Promise((resolve, reject) => {
            const request = store.get(key);
            request.onsuccess = () => {
                const result = request.result as StorageItem<T> | undefined;
                if (result && (!result.timeout || Date.now() - result.time < result.timeout)) {
                    resolve(result.value);
                } else {
                    resolve(null);
                }
            };
            request.onerror = () => reject(request.error);
        });
    }

    async removeItem(key: string): Promise<void> {
        const transaction = this.db.transaction(this.storeName, 'readwrite');
        const store = transaction.objectStore(this.storeName);
        return new Promise((resolve, reject) => {
            const request = store.delete(key);
            request.onsuccess = () => resolve();
            request.onerror = () => reject(request.error);
        });
    }
}
