import { Factory } from '../base';
import { reducer, initState, IInitState } from './reducer';
import { CombineAction } from './actions';
import { createLocalStoragePersistor } from '../base/Persistor';

export * from './actions';

export const { StoreProvider: PanelLayoutStoreProvider, useStore: usePanelLayoutStore } = Factory.createStoreProvider<
    IInitState,
    CombineAction
>({
    initialState: initState,
    reducer,
    persistor: createLocalStoragePersistor('dv-panel-layout-stoer'),
});

export function useMosaicId() {
    const [store] = usePanelLayoutStore();
    return store.mosaicId;
}
