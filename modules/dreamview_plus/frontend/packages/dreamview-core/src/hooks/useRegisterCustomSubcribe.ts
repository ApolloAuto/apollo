import { useCallback, useLayoutEffect, useRef } from 'react';
import { CustomizeEvent, useEventHandlersContext } from '../store/EventHandlersStore';

export default function useRegisterCustomSubcribe(eventName: string) {
    const eventHandlers = useEventHandlersContext();
    const { customizeSubs } = eventHandlers;
    const publishEventRef = useRef<CustomizeEvent>();
    const publishData = useCallback((newData: unknown) => {
        if (publishEventRef.current) {
            publishEventRef.current.publish(newData);
        }
    }, []);

    useLayoutEffect(() => {
        customizeSubs.reigisterCustomizeEvent(eventName);
        publishEventRef.current = customizeSubs.getCustomizeEvent(eventName);
    }, [eventHandlers, eventName]);

    return publishData;
}
