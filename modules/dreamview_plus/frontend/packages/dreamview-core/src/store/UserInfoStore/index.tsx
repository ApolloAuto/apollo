import { Factory } from '../base';
import { IInitState, CombineAction } from './actions';
import { initState, reducer } from './reducer';

export const { StoreProvider: UserInfoStoreProvider, useStore: useUserInfoStore } = Factory.createStoreProvider<
    IInitState,
    CombineAction
>({
    initialState: initState,
    reducer,
});
