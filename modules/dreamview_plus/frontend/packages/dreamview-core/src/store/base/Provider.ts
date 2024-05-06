import React from 'react';
import Store from './Store';
import { Persistor } from './Persistor';

export interface StoreProviderProps<S, A> extends React.PropsWithChildren {
    reducer: (state: S, action: A) => S;
    initialState: S;
    middleware?: ((store: Store<S, A>, next: (action: A) => void, action: A) => void)[];
    persistor?: Persistor<S>;
}
