import { Factory } from '@dreamview/dreamview-core/src/store/base';
import { IInitState, CombineAction } from './actions';
import { initState, reducer } from './reducer';

export const { StoreProvider: VizProvider, useStore: useVizStore } = Factory.createStoreProvider<
    IInitState,
    CombineAction
>({
    initialState: initState,
    reducer,
});
