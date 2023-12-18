import { Factory } from '../base';
import { CombineAction } from './actions';
import { IInitState, initState, reducer } from './reducer';

export const { StoreProvider: PanelInfoStoreProvider, useStore: usePanelInfoStore } = Factory.createStoreProvider<
    IInitState,
    CombineAction
>({
    initialState: initState,
    reducer,
});
