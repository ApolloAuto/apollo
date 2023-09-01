import { useCallback, useEffect } from 'react';
import { useEventHandlersContext } from '../store/EventHandlersStore';

export default function useFullScreen(panelId: string) {
    const eventHandlers = useEventHandlersContext();
    const { customizeSubs } = eventHandlers;
    const { setFilterKey } = eventHandlers.keydown;

    const enterFullScreen = useCallback(() => {
        customizeSubs.getCustomizeEvent(`full:screen:${panelId}`).publish(true);
    }, [customizeSubs, panelId]);
    const exitFullScreen = useCallback(() => {
        customizeSubs.getCustomizeEvent(`full:screen:${panelId}`).publish(false);
    }, [customizeSubs, panelId]);

    useEffect(() => {
        setFilterKey((_) => {
            exitFullScreen();
        }, 'escape');
    }, [setFilterKey, exitFullScreen]);

    return [enterFullScreen, exitFullScreen];
}
