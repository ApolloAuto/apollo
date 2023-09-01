import React, { useMemo, useState } from 'react';
import EventEmitter from 'eventemitter3';

const context = React.createContext(null);

export function usePanelInnerEventEmitter() {
    return React.useContext(context).ee;
}

export function PanelInnerEventEmitterProvider(prop: React.PropsWithChildren) {
    const contextValue = useMemo(
        () => ({
            ee: new EventEmitter(),
        }),
        [],
    );
    return <context.Provider value={contextValue}>{prop.children}</context.Provider>;
}
