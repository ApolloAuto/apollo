import { useCallback, useEffect, useRef } from 'react';
import { CustomizeEvent, useEventHandlersContext } from '../store/EventHandlersStore';

export default function useGetPublicFn(eventName: string) {
    const eventHandlers = useEventHandlersContext();
    const { customizeSubs } = eventHandlers;
    const updateChannelEventRef = useRef<CustomizeEvent>();
    const publish = useCallback((data: unknown) => {
        if (updateChannelEventRef.current) {
            updateChannelEventRef.current.publish(data);
        }
    }, []);

    useEffect(() => {
        updateChannelEventRef.current = customizeSubs.getCustomizeEvent(eventName);
    }, [eventName]);

    return publish;
}
