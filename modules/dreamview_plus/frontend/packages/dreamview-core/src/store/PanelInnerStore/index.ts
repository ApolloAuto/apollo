import { useMemo, Dispatch, SetStateAction, useState, useRef } from 'react';
import { Factory } from '../base';
import { reducer } from './reducer';
import * as TYPES from './actionTypes';
import { createLocalStoragePersistor } from '../base/Persistor';

export * from './actionTypes';

export const { StoreProvider: PanelInnerProvider, useStore: usePanelInnerStore } = Factory.createStoreProvider<
    any,
    TYPES.CombineAction
>({
    initialState: undefined,
    reducer,
    persistor: createLocalStoragePersistor('dv-panel-inner-stoer'),
});

function isFunc(prop: any) {
    return typeof prop === 'function';
}

export function usePanelPersistorState<T = unknown>(
    key: string,
    propInitState?: T | (() => T),
): [T, Dispatch<SetStateAction<T>>] {
    const [{ panelId, state }, dispatch] = usePanelInnerStore();
    function createPayload(nextValue: T) {
        return {
            type: TYPES.ACTIONS.UPDATE,
            payload: {
                panelId,
                key,
                nextValue,
            },
        };
    }

    const value = useMemo(() => {
        if (key in (state?.[panelId] || {})) {
            return state?.[panelId]?.[key];
        }
        const nextValue = (() => {
            if (isFunc(propInitState)) {
                return (propInitState as () => T)();
            }
            return propInitState as T;
        })();

        const payload = createPayload(nextValue);
        dispatch(payload);

        return nextValue;
    }, [state?.[panelId]?.[key]]);

    const dispatchAction = (action: SetStateAction<T>) => {
        const nextValue = (() => {
            if (typeof action === 'function') {
                return (action as (prevState: T) => T)(value);
            }
            return action as T;
        })();

        const payload = createPayload(nextValue);
        dispatch(payload);
    };

    return useMemo(() => [value, dispatchAction], [value]);
}
