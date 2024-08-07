import React, { PropsWithChildren, useEffect, useRef, useLayoutEffect } from 'react';
import { Factory } from '../base';
import { MainApi, OtherApi, PluginApi, StreamApi } from '../../services/api';
import { WebSocketManager, ConnectionStatusEnum, MetadataItem, SocketNameEnum } from '../../services/WebSocketManager';
import { CustomizeEvent, useEventHandlersContext, CustomEventTypes } from '../EventHandlersStore';
import * as TYPES from './actionTypes';
import { IInitState, initState, reducer } from './reducer';
import { StreamDataNames } from '../../services/api/types';

export const { StoreProvider, useStore } = Factory.createStoreProvider<IInitState, TYPES.CombineAction>({
    initialState: initState,
    reducer,
});

function WebSocketManagerInner(): React.ReactElement {
    const [store, dispatch] = useStore();
    const eventHandlers = useEventHandlersContext();
    const mainConnectionRef = useRef<CustomizeEvent>();
    const pluginConnectionRef = useRef<CustomizeEvent>();

    useLayoutEffect(() => {
        const { customizeSubs } = eventHandlers;
        customizeSubs.reigisterCustomizeEvent(CustomEventTypes.MainConnectedEvent);
        mainConnectionRef.current = customizeSubs.getCustomizeEvent('main:connection');

        customizeSubs.reigisterCustomizeEvent(CustomEventTypes.PluginConnectedEvent);
        pluginConnectionRef.current = customizeSubs.getCustomizeEvent('plugin:connection');

        store?.mainApi?.webSocketManager?.metadata$.subscribe((metadata) => {
            dispatch({ type: TYPES.ACTIONS.UPDATE_METADATA, payload: metadata });
        });
    }, [eventHandlers]);

    useEffect(() => {
        store.mainApi.webSocketManager.connectMain().subscribe((status) => {
            if (status === ConnectionStatusEnum.METADATA) {
                const metadata = store.mainApi.webSocketManager.getMetadata();
                if (metadata.find((item) => item.dataName === StreamDataNames.SIM_HMI_STATUS)) {
                    // 包含仿真模块，则触发连接仿真模块（仿真模块化改造）
                    store.otherApi.getSocketIns(SocketNameEnum.SIMULATION);
                }
                mainConnectionRef.current.publish('main:connection successful');
            }
        });
        store.pluginApi.webSocketManager.connectPlugin().subscribe((status) => {
            if (status === ConnectionStatusEnum.CONNECTED) {
                pluginConnectionRef.current.publish('plugin:connection successful');
            }
        });

        return () => {
            store.mainApi.webSocketManager.disconnect();
            store.pluginApi.webSocketManager.disconnect();
        };
    }, [store.mainApi, store.pluginApi]);

    return null;
}

export function WebSocketManagerProvider(props: PropsWithChildren<{}>): React.ReactElement {
    return (
        <StoreProvider>
            <WebSocketManagerInner />
            {props.children}
        </StoreProvider>
    );
}

export function useMainApi(): MainApi {
    const [store] = useStore();
    return store?.mainApi;
}

export function usePluginApi(): PluginApi {
    const [store] = useStore();
    return store?.pluginApi;
}

export function useStreamApi(): StreamApi {
    const [store] = useStore();
    return store?.streamApi;
}

export function useOtherApi(): OtherApi {
    const [store] = useStore();
    return store?.otherApi;
}

export function useMetadata(): [MetadataItem[], (metadata: MetadataItem[]) => void] {
    const [store] = useStore();
    return [
        store?.metadata,
        (metadata) => {
            store?.mainApi?.webSocketManager?.setMetadata(metadata);
        },
    ];
}

export function useWebSocketManager(): WebSocketManager {
    const [store] = useStore();
    return store?.mainApi?.webSocketManager;
}
