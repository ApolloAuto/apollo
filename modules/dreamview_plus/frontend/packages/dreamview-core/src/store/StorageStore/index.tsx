import React from 'react';

export type IStorageContext<T> = {
    nameScape: string;
    update: (key: string, value: any) => Promise<T>;
    get: (key: string) => Promise<T>;
    remove: (key: string) => Promise<T>;
};

export function generateStorageContext<T>(func: IStorageContext<T>) {
    const storageContext = React.createContext<IStorageContext<T>>(null);

    function Provider(subProps: React.PropsWithChildren) {
        return <storageContext.Provider value={func}>{subProps.children}</storageContext.Provider>;
    }
    function useStorage() {
        return React.useContext(storageContext);
    }

    return {
        storageContext,
        StorageProvider: Provider,
        useStorage,
    };
}
