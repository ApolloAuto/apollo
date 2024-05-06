import { Factory } from '../base';
import { reducer, initState, IInitState, mosaicId } from './reducer';
import { CombineAction } from './actions';
import { createLocalStoragePersistor } from '../base/Persistor';
import { CURRENT_MODE, usePickHmiStore } from '../HmiStore';

export * from './actions';

export const { StoreProvider: PanelLayoutStoreProvider, useStore: usePanelLayoutStore } = Factory.createStoreProvider<
    IInitState,
    CombineAction
>({
    initialState: initState,
    reducer,
    persistor: createLocalStoragePersistor('dv-panel-layout-stoer-v4'),
});

export function useMosaicId() {
    const [store] = usePanelLayoutStore();
    const [hmi] = usePickHmiStore();
    return store.layout[hmi.currentMode]?.mosaicId || mosaicId;
}

export function useGetCurrentLayout() {
    const [hmi] = usePickHmiStore();
    const [store] = usePanelLayoutStore();

    return store.layout[hmi.currentMode]?.layout;
}
