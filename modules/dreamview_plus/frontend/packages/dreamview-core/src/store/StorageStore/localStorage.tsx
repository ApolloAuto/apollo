import { generateStorageContext, IStorageContext } from './index';

export class LocalStorage<T> implements IStorageContext<T> {
    public nameScape: string;

    constructor(nameScape: string) {
        this.nameScape = nameScape;
    }

    private getStorageKey = (key: string) => `${this.nameScape}-${key}`;

    private getFromLocal = (key: string): Promise<T> =>
        new Promise((resolve) => {
            const str = localStorage.getItem(this.getStorageKey(key));
            if (str) {
                try {
                    const wrapper = JSON.parse(str);
                    resolve(wrapper.value);
                } catch (e) {
                    resolve(null);
                }
            } else {
                resolve(null);
            }
        });

    public update = (key: string, value: any): Promise<T> => {
        localStorage.setItem(
            this.getStorageKey(key),
            JSON.stringify({
                time: Date.now(),
                value,
            }),
        );
        return this.getFromLocal(key);
    };

    public get = (key: string): Promise<T> => this.getFromLocal(key);

    public remove(key: string): Promise<T> {
        localStorage.removeItem(key);
        return this.getFromLocal(key);
    }
}

export const { StorageProvider, storageContext } = generateStorageContext(new LocalStorage('locals'));
