import { useEffect, useRef } from 'react';
import { CustomizeEvent, useEventHandlersContext } from '../store/EventHandlersStore';

export default function useUpdateChannel(panelId: string, updateChannel: (newChannel: any) => void) {
    const eventHandlers = useEventHandlersContext();
    const { customizeSubs } = eventHandlers;
    const eventRef = useRef<CustomizeEvent>();

    useEffect(() => {
        eventRef.current = customizeSubs.getCustomizeEvent(`channel:update:${panelId}`);
        if (eventRef.current) {
            eventRef.current.subscribe(updateChannel);
        }
    }, [updateChannel]);

    useEffect(
        () => () => {
            if (eventRef.current) {
                eventRef.current.removeSubscribe();
            }
        },
        [],
    );

    return eventRef.current;
}
