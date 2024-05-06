import React, { createContext, useMemo, useContext, useEffect, useState } from 'react';
import Store from './Store';
import { StoreProviderProps } from './Provider';
import {
    loggerMiddleware,
    asyncActionMiddleware,
    crashReporterMiddleware,
    reduxDevToolsMiddleware,
} from './Middleware';

export function createStoreProvider<InitialStateType, ActionType>({
    initialState,
    reducer,
    middleware = [],
    persistor,
}: StoreProviderProps<InitialStateType, ActionType>) {
    const StoreContext = createContext<Store<InitialStateType, ActionType>>(null);

    const middlewareExtra = [
        asyncActionMiddleware,
        // crashReporterMiddleware,
        // reduxDevToolsMiddleware,
    ];

    const StoreProvider: React.FC<Partial<StoreProviderProps<InitialStateType, ActionType>>> = function (props) {
        const iPersistor = props.persistor || persistor;

        const store = useMemo(() => {
            // 优先级： persistor.loadSync() > {} > initialState > props.initialState
            let initState = props.initialState || initialState;
            if (iPersistor && 'loadSync' in iPersistor && 'saveSync' in iPersistor) {
                // If synchronous persistence method exists, use it to load the initial state
                initState = iPersistor.loadSync() || initState;
            }
            return new Store(initState, props.reducer || reducer);
        }, []);

        store.applyMiddleware(...(props.middleware || middleware), ...middlewareExtra);

        if (iPersistor) {
            store.persist(iPersistor);
        }

        return <StoreContext.Provider value={store}>{props.children}</StoreContext.Provider>;
    };

    function useStore(): [InitialStateType, (action: ActionType) => void] {
        const store = useContext(StoreContext);

        if (!store) {
            throw new Error('useStore must be used within a StoreProvider');
        }

        const [state, setState] = useState(store.getState());

        useEffect(() => {
            const subscription = store.subscribe(setState);
            return () => subscription.unsubscribe();
        }, [store]);

        return [state, store.dispatch];
    }

    return { StoreProvider, useStore, StoreContext };
}
