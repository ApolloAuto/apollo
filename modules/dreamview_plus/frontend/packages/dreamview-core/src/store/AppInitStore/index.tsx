import React, { PropsWithChildren, useCallback, useEffect, useState } from 'react';
import shortUUID from 'short-uuid';
import useOnce from '../../hooks/useOnce';

export interface IAppInitContext {
    initStatus: boolean;
    registry: (uid: string) => void;
    onInit: (uid: string) => void;
}

export const AppInitContext = React.createContext(null);

export function useAppInitContext() {
    return React.useContext(AppInitContext);
}

export enum IAppInitStatus {
    INIT,
    DONE,
    FAIL,
    LOADING,
}

type IRegistryItem = { init: IAppInitStatus; moduleName: string; progress?: number; message?: string };

type IRegistry = Record<string, IRegistryItem>;

/**
 * collect child components init status
 */
export function AppInitProvider(props: PropsWithChildren) {
    const [initStatus, setInitStatus] = React.useState<IAppInitStatus>(IAppInitStatus.INIT);
    const [hasInit, setHasInit] = useState(false);
    const registry = React.useRef<IRegistry>({});

    const onChildStatusChange = (iregistry: IRegistry) => {
        const registryCollection = Object.values(iregistry);
        const hasAllFinished = registryCollection.every((item) =>
            [IAppInitStatus.FAIL, IAppInitStatus.DONE].includes(item.init),
        );
        if (!hasAllFinished) {
            return;
        }
        setHasInit(true);
        const hasSomeChildLoadFail = registryCollection.some((item) => item.init === IAppInitStatus.FAIL);
        if (hasSomeChildLoadFail) {
            setInitStatus(IAppInitStatus.FAIL);
        } else {
            setInitStatus(IAppInitStatus.DONE);
        }
    };

    const values = React.useMemo(
        () => ({
            hasInit,
            initStatus,
            registry: (uid: string, moduleName: string) => {
                registry.current[uid] = {
                    init: IAppInitStatus.INIT,
                    moduleName,
                };
            },
            onChange: (
                uid: string,
                value: { init: IAppInitStatus; moduleName: string; progress?: number; message?: string },
            ) => {
                registry.current[uid] = value;
                onChildStatusChange(registry.current);
            },
        }),
        [initStatus, hasInit],
    );

    useEffect(() => {
        const registryEventLength = Object.keys(registry.current);
        const hasRegistryEvent = !!registryEventLength;
        if (!hasRegistryEvent) {
            setHasInit(true);
            setInitStatus(IAppInitStatus.DONE);
        }
    }, []);

    return <AppInitContext.Provider value={values}>{props.children}</AppInitContext.Provider>;
}

export type IInjectAppInitStatus<T> = T & { onInitStatusChange: (value: IRegistry) => void };

export function useRegistryInitEvent(moduleName: string) {
    const { registry, onChange } = useAppInitContext();
    const [uid] = React.useState(() => shortUUID.generate());

    useOnce(() => {
        registry(uid, moduleName);
    });

    return useCallback((status: Omit<IRegistryItem, 'moduleName'>) => {
        onChange(uid, {
            moduleName,
            ...status,
        });
    }, []);
}
