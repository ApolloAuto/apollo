import { useCallback, useLayoutEffect, useRef } from 'react';
import { CustomizeEvent, useEventHandlersContext } from '../store/EventHandlersStore';
import { SubscribeInfo } from '../components/panels/type/RenderToolBar';

export default function useRegisterUpdateChanel(panelId: string) {
    const eventHandlers = useEventHandlersContext();
    const { customizeSubs } = eventHandlers;
    const updateChannelEventRef = useRef<CustomizeEvent>();
    const updateChannel = useCallback((newChannel: SubscribeInfo) => {
        if (updateChannelEventRef.current) {
            updateChannelEventRef.current.publish(newChannel);
        }
    }, []);

    useLayoutEffect(() => {
        customizeSubs.reigisterCustomizeEvent(`channel:update:${panelId}`);
        updateChannelEventRef.current = customizeSubs.getCustomizeEvent(`channel:update:${panelId}`);
    }, [eventHandlers, panelId]);

    return updateChannel;
}
