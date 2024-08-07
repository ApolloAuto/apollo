import { useEffect, useState } from 'react';
import {
    useWebSocketManager,
    useMainApi,
    usePluginApi,
    useStreamApi,
    useMetadata,
    useOtherApi,
} from '@dreamview/dreamview-core/src/store/WebSocketManagerStore';
import { CustomEventTypes } from '@dreamview/dreamview-core/src/store/EventHandlersStore/eventType';
import { useEventHandlersContext } from '../../store/EventHandlersStore';
import { MetadataItem } from '../WebSocketManager';
import { MainApi, OtherApi, PluginApi, StreamApi } from '../api';
import { Nullable } from '../../util/similarFunctions';

type useWebSocketServicesReturnType = {
    isMainConnected: boolean;
    isPluginConnected: boolean;
    metadata: MetadataItem[];
    mainApi: Nullable<MainApi>;
    pluginApi: Nullable<PluginApi>;
    streamApi: Nullable<StreamApi>;
    otherApi: Nullable<OtherApi>;
    setMetaData: (metadata: MetadataItem[]) => void;
};

export default function useWebSocketServices(): useWebSocketServicesReturnType {
    const mainApiIns = useMainApi();

    const streamApiIns = useStreamApi();

    const pluginApiIns = usePluginApi();

    const otherApiIns = useOtherApi();

    const [metadata, setMetaData] = useMetadata();

    const webSocketManager = useWebSocketManager();

    const eventHandlers = useEventHandlersContext();

    const { customizeSubs } = eventHandlers;

    const [isMainConnected, setMainConnected] = useState(false);

    const [isPluginConnected, setPluginConnected] = useState(false);

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

    return {
        isMainConnected,
        isPluginConnected,
        metadata,
        mainApi: mainApiIns,
        streamApi: streamApiIns,
        pluginApi: pluginApiIns,
        otherApi: otherApiIns,
        setMetaData,
    };
}
