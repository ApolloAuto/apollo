export class LocalStorage<T> {
    // eslint-disable-next-line @typescript-eslint/no-var-requires,global-require
    defaultVersion = require('@dreamview/dreamview-core/package.json').version;

    version: string;

    storageKey: string;

    constructor(key: string, version?: string) {
        this.storageKey = key;
        this.version = version || this.defaultVersion;
    }

    private ifTimeExpire = (time: any) => {
        if (!time) {
            return false;
        }

        const now = Date.now();
        return now > new Date(time).getTime();
    };

    set = (value: T, others?: { timeout?: number }) => {
        localStorage.setItem(
            this.storageKey,
            JSON.stringify({
                timeout: others?.timeout,
                version: this.version,
                value,
            }),
        );
    };

    /**
     * 获取指定storage
     * @param {string} key
     */
    get = (defaultVal?: T) => {
        const str = localStorage.getItem(this.storageKey);
        if (str) {
            try {
                const wrapper = JSON.parse(str) || {};
                const { timeout, version: currentVersion } = wrapper;

                if (this.ifTimeExpire(timeout)) {
                    return defaultVal;
                }

                const isVersionExperied = this.version !== currentVersion;
                if (isVersionExperied) {
                    return defaultVal;
                }

                return wrapper.value;
            } catch (e) {
                return defaultVal;
            }
        }
        return defaultVal;
    };

    /**
     * 移除指定storage
     * @param {string} key
     */
    remove = () => {
        localStorage.removeItem(this.storageKey);
    };
}
