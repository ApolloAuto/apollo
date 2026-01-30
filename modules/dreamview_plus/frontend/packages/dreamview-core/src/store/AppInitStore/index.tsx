import React, { PropsWithChildren, useCallback, useEffect, useState, useRef } from 'react';
import shortUUID from 'short-uuid';
import useOnce from '../../hooks/useOnce';

export enum IAppInitStatus {
    DONE = 'DONE',
    FAIL = 'FAIL',
    LOADING = 'LOADING',
}

export type IRegistryItem = {
    status: IAppInitStatus;
    moduleName: string;
    progress?: number;
    message?: string;
    order?: number;
};

export type IRegistry = Record<string, IRegistryItem>;
export interface IAppInitContext {
    hasInit: boolean;
    initStatus: IAppInitStatus;
    registry: (uid: string, name: string, order: number) => void;
    onChange: (
        uid: string,
        value: { order?: number; status: IAppInitStatus; moduleName: string; progress?: number; message?: string },
    ) => void;
    progressChangeHandler: (callback: any) => void;
}

export const AppInitContext = React.createContext<IAppInitContext>(null);

export function useAppInitContext() {
    return React.useContext(AppInitContext);
}

/**
 * collect child components init status
 */
export function AppInitProvider(props: PropsWithChildren) {
    const [initStatus, setInitStatus] = useState<IAppInitStatus>(IAppInitStatus.DONE);
    const [hasInit, setHasInit] = useState(false);
    const registryRef = useRef<IRegistry>({});
    const progressChange = useRef([]);
    const onChildStatusChange = (iregistry: IRegistry) => {
        const registryCollection = Object.values(iregistry);
        const hasAllFinished = registryCollection.every((item) =>
            [IAppInitStatus.FAIL, IAppInitStatus.DONE].includes(item.status),
        );
        if (!hasAllFinished) {
            return;
        }
        setTimeout(() => {
            setHasInit(true);
        }, 500);
        const hasSomeChildLoadFail = registryCollection.some((item) => item.status === IAppInitStatus.FAIL);
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
            registry: (uid: string, moduleName: string, order: number) => {
                registryRef.current[uid] = {
                    status: IAppInitStatus.LOADING,
                    moduleName,
                    order,
                };
            },
            onChange: (
                uid: string,
                value: { status: IAppInitStatus; moduleName: string; progress?: number; message?: string },
            ) => {
                registryRef.current[uid] = value;
                onChildStatusChange(registryRef.current);
                progressChange.current.forEach((callback) => {
                    callback({ ...registryRef.current });
                });
            },
            progressChangeHandler: (callback: any) => {
                progressChange.current.push(callback);
                callback({ ...registryRef.current });
            },
        }),
        [initStatus, hasInit],
    );

    useEffect(() => {
        const registryEventLength = Object.keys(registryRef.current).length;
        const hasRegistryEvent = !!registryEventLength;
        if (!hasRegistryEvent) {
            setHasInit(true);
            setInitStatus(IAppInitStatus.DONE);
        }
    }, []);

    useEffect(() => {
        if (hasInit) {
            progressChange.current = [];
        }
    }, [hasInit]);

    return <AppInitContext.Provider value={values}>{props.children}</AppInitContext.Provider>;
}

export type IInjectAppInitStatus<T> = T & { onInitStatusChange: (value: IRegistry) => void };

export function useRegistryInitEvent(moduleName: string, order?: number) {
    const { registry, onChange } = useAppInitContext();
    const [uid] = React.useState(() => shortUUID.generate());

    useOnce(() => {
        registry(uid, moduleName, order);
    });

    return useCallback((status: Omit<IRegistryItem, 'moduleName' | 'order'>) => {
        onChange(uid, {
            moduleName,
            order,
            ...status,
        });
    }, []);
}
