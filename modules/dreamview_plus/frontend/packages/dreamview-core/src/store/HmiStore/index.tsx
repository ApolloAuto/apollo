import React, { useContext, useEffect, useMemo, useRef, useState } from 'react';
import isEqual from 'lodash/isEqual';
import { Factory } from '../base';
import { reducer, initState } from './reducer';
import { CombineAction } from './actions';
import { IInitState } from './actionTypes';

export * from './actionTypes';
export * from './actions';

export { hmiUtils } from './reducerHandler';

export const THROTTLE_TIME = 3000;

export const {
    StoreProvider: HmiStoreProvider,
    useStore: useHmiStore,
    StoreContext,
} = Factory.createStoreProvider<IInitState, CombineAction>({
    initialState: initState,
    reducer,
});

const pickHmiStoreContext = React.createContext(null);

export function usePickHmiStore(): [IInitState, (action: CombineAction) => void] {
    const store = useContext(pickHmiStoreContext);

    if (!store) {
        throw new Error('useStore must be used within a StoreProvider');
    }
    return store;
}

type hmiKeys = keyof IInitState;
interface PickHmiStoreProviderProps {
    keys?: hmiKeys[];
}

// 将hmi中频繁变更的状态剔除，留下变更不频繁的状态，减少由于状态变更导致的额外计算。
const defaultKeys = Object.keys(initState).filter(
    (item) => !['currentRecordStatus', 'prevStatus'].includes(item),
) as hmiKeys[];
function useDeepEffect(callback: any, effect: any) {
    const prev = useRef(null);
    useEffect(() => {
        if (!isEqual(prev.current, effect)) {
            callback();
        }
        prev.current = effect;
    }, effect);
}
export function PickHmiStoreProvider(props: React.PropsWithChildren<PickHmiStoreProviderProps>) {
    const { keys: propKeys = defaultKeys } = props;
    const [hmi, dispatch] = useHmiStore();
    // 排除意外导致keys引用变更的情况
    const [keys] = useState<hmiKeys[]>(propKeys);
    const [state, setState] = useState(
        () => keys.reduce((result, key) => ({ ...result, [key]: hmi[key] }), {}) as IInitState,
    );

    const isLoad = useRef(true);
    const dependence = keys.map((key) => hmi[key]);
    useDeepEffect(() => {
        if (isLoad.current) {
            isLoad.current = false;
            return;
        }
        setState(() => keys.reduce((result, key) => ({ ...result, [key]: hmi[key] }), {}) as IInitState);
    }, dependence);

    const context = useMemo(() => [state, dispatch], [state, dispatch]);

    return <pickHmiStoreContext.Provider value={context}>{props.children}</pickHmiStoreContext.Provider>;
}
