/**
 * 设置storage
 * @param {string} key
 * @param {any} value
 */
export const setLocalStorageItem = (key: string, value: any, timeout?: number) => {
    localStorage.setItem(
        key,
        JSON.stringify({
            time: Date.now(),
            timeout,
            value,
        }),
    );
};

/**
 * 移除指定storage
 * @param {string} key
 */
export const removeLocalStorage = (key: string) => {
    localStorage.removeItem(key);
};

/**
 * 获取指定storage
 * @param {string} key
 */
export const getLocalStorageItem = (key: string) => {
    const str = localStorage.getItem(key);
    if (str) {
        try {
            const wrapper = JSON.parse(str);
            return wrapper.value;
        } catch (e) {
            return null;
        }
    }
    return null;
};

/**
 * 移除指定storage
 * @param {string} namescape
 */
export const removeLocalStorageByNameScape = (namescape: string) => {
    const all = localStorage.valueOf();
    Object.keys(all).forEach((key) => {
        if (key.indexOf(namescape) === 0) {
            removeLocalStorage(key);
        }
    });
};

export default class PersistStorage {
    storageKey: string;

    constructor(key: string) {
        this.storageKey = key;
    }

    set = (key: string, value: any, timeout?: number) => {
        localStorage.setItem(
            key,
            JSON.stringify({
                time: Date.now(),
                value,
            }),
        );
    };

    /**
     * 获取指定storage
     * @param {string} key
     */
    get = (key: string) => {
        const str = localStorage.getItem(key);
        if (str) {
            try {
                const wrapper = JSON.parse(str);
                return wrapper.value;
            } catch (e) {
                return null;
            }
        }
        return null;
    };

    /**
     * 移除指定storage
     * @param {string} key
     */
    remove = (key: string) => {
        localStorage.removeItem(key);
    };

    /**
     * 移除指定storage
     * @param {string} namescape
     */
    removeLocalStorageByNameScape = (namescape: string) => {
        const all = localStorage.valueOf();
        Object.keys(all).forEach((key) => {
            if (key.indexOf(namescape) === 0) {
                removeLocalStorage(key);
            }
        });
    };
}
