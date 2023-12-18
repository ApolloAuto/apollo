import { Factory } from '../base';
import { reducer, initState, IInitState } from './reducer';
import { CombineAction } from './actions';

export * from './reducerHandler';

export * from './actionTypes';

export const { StoreProvider: MenuStoreProvider, useStore: useMenuStore } = Factory.createStoreProvider<
    IInitState,
    CombineAction
>({
    initialState: initState,
    reducer,
});
