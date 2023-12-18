import React, { PropsWithChildren, useContext, useMemo, useState } from 'react';

const context = React.createContext(null);

export function useProfileManagerContext() {
    return useContext(context);
}

export enum ENUM_PROFILEMANAGER_TAB {
    Records = 'Records',
    Scenarios = 'Scenarios',
    HDMap = 'HDMap',
    Vehicle = 'Vehicle',
    V2X = 'V2X',
    Dynamical = 'Dynamical Model',
}

export function ProfileManagerProvider(props: PropsWithChildren) {
    const [activeTab, setTab] = useState<ENUM_PROFILEMANAGER_TAB>(ENUM_PROFILEMANAGER_TAB.Records);
    const [filter, setFilter] = useState({ downLoadStatus: 'all' });
    const value = useMemo(
        () => ({
            filter,
            setFilter,
            activeTab,
            setTab,
        }),
        [filter, activeTab],
    );
    return <context.Provider value={value}>{props.children}</context.Provider>;
}
