import pick from 'lodash/pick';
import omit from 'lodash/omit';
import { LocalStorage, KEY_MANAGER } from '@dreamview/dreamview-core/src/util/storageManager';

export type Persistor<S> =
    | {
          // fixme: 异步还未实现，可能导致store先init后出发修改的操作。
          load: () => S | Promise<S>;
          save: (state: S) => void | Promise<void>;
      }
    | { loadSync?: () => S; saveSync?: (state: S) => void };

type PersistorOptions = { pick: string[]; omit?: never } | { pick?: never; omit: string[] };

function handleOptions<S>(state: Partial<S>, options: PersistorOptions): Partial<S> {
    let resultState: Partial<S>;
    if (options.pick) {
        resultState = pick(state, options.pick);
    } else if (options.omit) {
        resultState = omit(state, options.omit);
    } else {
        resultState = state;
    }
    return resultState;
}

/**
 * 高阶函数，用于包装某个Persistor，实现对state的pick或omit，局部持久化
 * Usage:
 * const customConfigurablePersistor = createConfigurablePersistor<{ a: string; b: string }>(
 *     createLocalStoragePersistor('dreamview'),
 *     {
 *         pick: ['a'],
 *     },
 * );
 * @param persistor 持久化方法对象
 * @param options 选项
 */
export function createConfigurablePersistor<S>(
    persistor: Persistor<Partial<S>>,
    options: PersistorOptions,
): Persistor<Partial<S>> {
    if (options.pick && options.omit) {
        throw new Error('Cannot use both "pick" and "omit" options at the same time.');
    }

    if ('load' in persistor && 'save' in persistor) {
        return {
            load: async () => {
                const state = await persistor.load();
                return handleOptions<S>(state, options);
            },
            save: async (state) => {
                const stateToPersist = handleOptions<S>(state, options);
                await persistor.save(stateToPersist);
            },
        };
    }
    if ('loadSync' in persistor && 'saveSync' in persistor) {
        return {
            loadSync: () => {
                const state = persistor.loadSync();
                return handleOptions<S>(state, options);
            },
            saveSync: (state) => {
                const stateToPersist = handleOptions<S>(state, options);
                persistor.saveSync(stateToPersist);
            },
        };
    }
    throw new Error('Invalid Persistor: must have either load/save or loadSync/saveSync');
}

const dreamviewStorageManager = new LocalStorage(KEY_MANAGER.DV);
export const localStoragePersistor: Persistor<any> = {
    loadSync: () => dreamviewStorageManager.get(),
    saveSync: (state) => dreamviewStorageManager.set(state),
};

// 用于生成localStoragePersistor
export function createLocalStoragePersistor<S>(key: string): Persistor<Partial<S>> {
    const sotrageManager = new LocalStorage(key);
    return {
        loadSync: () => sotrageManager.get(),
        saveSync: (state) => sotrageManager.set(state),
    };
}
