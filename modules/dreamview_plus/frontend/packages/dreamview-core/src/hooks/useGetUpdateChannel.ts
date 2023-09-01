import { useCallback, useEffect, useRef } from 'react';
import { CustomizeEvent, useEventHandlersContext } from '../store/EventHandlersStore';
import { SubscribeInfo } from '../components/panels/type/RenderToolBar';

export default function useGetUpdateChannel(panelId: string) {
    const eventHandlers = useEventHandlersContext();
    const { customizeSubs } = eventHandlers;
    const updateChannelEventRef = useRef<CustomizeEvent>();
    const updateChannel = useCallback((newChannel: SubscribeInfo) => {
        if (updateChannelEventRef.current) {
            updateChannelEventRef.current.publish(newChannel);
        }
    }, []);

    useEffect(() => {
        updateChannelEventRef.current = customizeSubs.getCustomizeEvent(`channel:update:${panelId}`);
    }, [panelId]);

    return updateChannel;
}
