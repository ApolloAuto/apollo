import { useEffect, useRef } from 'react';
import { CustomizeEvent, useEventHandlersContext } from '../store/EventHandlersStore';

export default function useNotifyInitialChanel(panelId: string, addChannel: (newChannel: any) => void) {
    const eventHandlers = useEventHandlersContext();
    const { customizeSubs } = eventHandlers;
    const eventRef = useRef<CustomizeEvent>();

    useEffect(() => {
        eventRef.current = customizeSubs.getCustomizeEvent(`channel:notify:${panelId}`);
        if (eventRef.current) {
            eventRef.current.subscribe(addChannel);
        }
    }, [addChannel]);

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
