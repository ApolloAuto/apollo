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
