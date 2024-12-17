import { useRef } from 'react';
import { Factory } from '../base';
import { reducer, IInitState, mosaicId } from './reducer';
import { CombineAction } from './actions';
import { usePickHmiStore } from '../HmiStore';

export * from './actions';

export const { StoreProvider: PanelLayoutStoreProvider, useStore: usePanelLayoutStore } = Factory.createStoreProvider<
    IInitState,
    CombineAction
>({
    initialState: {
        layout: {} as IInitState['layout'],
        initLatout: {} as IInitState['initLatout'],
    },
    reducer,
});

export function useMosaicId() {
    const [store] = usePanelLayoutStore();
    const [hmi] = usePickHmiStore();
    return store.layout[hmi.currentMode]?.mosaicId || mosaicId;
}

/**
 * @function useGetCurrentLayout
 * @description 使用useGetCurrentLayout函数获取当前布局。
 * 该函数从pickHmiStore和panelLayoutStore中获取相应的状态，并返回当前模式下的布局。
 * 如果当前布局为空，则返回上一次的布局。
 *
 * @returns {string | null} currentLayout - 当前布局，可能为字符串或null
 */
export function useGetCurrentLayout() {
    const [hmi] = usePickHmiStore();
    const [store] = usePanelLayoutStore();
    const prevLayout = useRef(null);
    const currentLayout = store.layout[hmi.currentMode]?.layout;
    if (!currentLayout) {
        return prevLayout.current;
    }

    prevLayout.current = currentLayout;
    return currentLayout;
}
