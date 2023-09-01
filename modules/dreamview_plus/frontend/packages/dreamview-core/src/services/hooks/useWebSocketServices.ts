import { useEffect, useMemo, useState } from 'react';
import {
    useWebSocketManager,
    useMainApi,
    usePluginApi,
    useStreamApi,
    useMetadata,
} from '@dreamview/dreamview-core/src/store/WebSocketManagerStore';
import { useEventHandlersContext } from '../../store/EventHandlersStore';
import { CustomEventTypes } from '../../store/EventHandlersStore/eventType';
import { MetadataItem } from '../WebSocketManager/type';
import { MainApi, PluginApi, StreamApi } from '../api';
import { Nullable } from '../../util/similarFunctions';

type useWebSocketServicesReturnType = {
    isMainConnected: boolean;
    isPluginConnected: boolean;
    metadata: MetadataItem[];
    mainApi: Nullable<MainApi>;
    pluginApi: Nullable<PluginApi>;
    streamApi: Nullable<StreamApi>;
    setMetaData: (metadata: MetadataItem[]) => void;
};

export default function useWebSocketServices(): useWebSocketServicesReturnType {
    const mainApiIns = useMainApi();

    const streamApiIns = useStreamApi();

    const pluginApiIns = usePluginApi();

    const [metadata, setMetaData] = useMetadata();

    const webSocketManager = useWebSocketManager();

    const eventHandlers = useEventHandlersContext();

    const { customizeSubs } = eventHandlers;

    const [isMainConnected, setMainConnected] = useState(false);

    const [isPluginConnected, setPluginConnected] = useState(false);

    const mainApi = useMemo(() => {
        if (!isMainConnected) return null;
        return mainApiIns;
    }, [isMainConnected]);

    const streamApi = useMemo(() => {
        if (!isMainConnected) return null;
        return streamApiIns;
    }, [isMainConnected]);

    const pluginApi = useMemo(() => {
        if (!isPluginConnected) return null;
        return pluginApiIns;
    }, [isPluginConnected]);

    useEffect(() => {
        customizeSubs.getCustomizeEvent(CustomEventTypes.MainConnectedEvent).subscribe(() => {
            setMainConnected(webSocketManager.isMainConnected());
            setMetaData(webSocketManager.getMetadata());
        });

        customizeSubs.getCustomizeEvent(CustomEventTypes.PluginConnectedEvent).subscribe(() => {
            setPluginConnected(webSocketManager.isPluginConnected());
        });

        return () => {
            setMainConnected(false);
            setPluginConnected(false);
        };
    }, [customizeSubs, webSocketManager]);

    return { isMainConnected, isPluginConnected, metadata, mainApi, streamApi, pluginApi, setMetaData };
}
