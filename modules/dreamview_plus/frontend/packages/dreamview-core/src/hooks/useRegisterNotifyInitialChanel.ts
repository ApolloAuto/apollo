import { useCallback, useLayoutEffect, useRef } from 'react';
import { CustomizeEvent, useEventHandlersContext } from '../store/EventHandlersStore';
import { SubscribeInfo } from '../components/panels/type/RenderToolBar';

export default function useRegisterNotifyInitialChanel(panelId: string) {
    const eventHandlers = useEventHandlersContext();
    const { customizeSubs } = eventHandlers;
    const updateChannelEventRef = useRef<CustomizeEvent>();
    const notifyInitialChannel = useCallback((newChannel: SubscribeInfo) => {
        if (updateChannelEventRef.current) {
            updateChannelEventRef.current.publish(newChannel);
        }
    }, []);

    useLayoutEffect(() => {
        customizeSubs.reigisterCustomizeEvent(`channel:notify:${panelId}`);
        updateChannelEventRef.current = customizeSubs.getCustomizeEvent(`channel:notify:${panelId}`);
    }, [eventHandlers, panelId]);

    return notifyInitialChannel;
}
