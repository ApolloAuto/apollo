import { useCallback, useMemo, useState } from 'react';
import { LocalStorage } from './LocalStorage';

export function useLocalStorage<T>(propKey: string, version?: string): LocalStorage<T> {
    const [storageManager] = useState(() => new LocalStorage(propKey, version));

    return storageManager;
}

export function useLocalStorageState<T>(propKey: string, defaultVal?: T, version?: string) {
    const storageManager = useLocalStorage(propKey, version);
    const [state, setState] = useState(() => storageManager.get() || defaultVal);

    const mySetState = useCallback((val: T | ((prop: T) => T), timeout?: number) => {
        if (typeof val === 'function') {
            setState((prev: T) => {
                const next = (val as (propVal: T) => T)(prev);
                storageManager.set(next, { timeout });
                return next;
            });
        } else {
            storageManager.set(val, { timeout });
            setState(val);
        }
    }, []);

    return useMemo(() => [state, mySetState], [state, mySetState]);
}
