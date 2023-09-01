import { useEffect, useRef } from 'react';
import { CustomizeEvent, useEventHandlersContext } from '../store/EventHandlersStore';

export default function useCustomSubcribe(eventName: string, subscribeCallback: (data: any) => void) {
    const eventHandlers = useEventHandlersContext();
    const { customizeSubs } = eventHandlers;
    const eventRef = useRef<CustomizeEvent>();

    useEffect(() => {
        eventRef.current = customizeSubs.getCustomizeEvent(eventName);
        if (eventRef.current) {
            eventRef.current.subscribe(subscribeCallback);
        }

        return () => {
            if (eventRef.current) {
                eventRef.current.removeSubscribe();
            }
        };
    }, []);

    return eventRef.current;
}
