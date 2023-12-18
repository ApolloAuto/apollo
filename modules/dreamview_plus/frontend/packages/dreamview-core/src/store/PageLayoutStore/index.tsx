import { Factory } from '../base';
import { reducer, initState, IInitState } from './reducer';
import { CombineAction } from './actions';
import { createLocalStoragePersistor } from '../base/Persistor';

export const { StoreProvider: PageLayoutStoreProvider, useStore: usePageLayoutStore } = Factory.createStoreProvider<
    IInitState,
    CombineAction
>({
    initialState: initState,
    reducer,
    persistor: createLocalStoragePersistor('pageLayoutStore'),
});
