import { useCallback, useEffect } from 'react';
import { useEventHandlersContext } from '../store/EventHandlersStore';
import { hooksManager } from '../util/HooksManager';

export default function useFullScreen(panelId: string) {
    const eventHandlers = useEventHandlersContext();
    const { customizeSubs } = eventHandlers;

    const enterFullScreen = useCallback(() => {
        const fullScreenHookConfig = hooksManager.getHook(panelId);

        if (fullScreenHookConfig?.beforeEnterFullScreen) {
            fullScreenHookConfig?.beforeEnterFullScreen();
        }

        customizeSubs.getCustomizeEvent(`full:screen:${panelId}`).publish(true);

        if (fullScreenHookConfig?.afterEnterFullScreen) {
            fullScreenHookConfig?.afterEnterFullScreen();
        }
    }, [customizeSubs, panelId]);
    const exitFullScreen = useCallback(() => {
        const fullScreenHookConfig = hooksManager.getHook(panelId);

        if (fullScreenHookConfig?.beforeExitFullScreen) {
            fullScreenHookConfig?.beforeExitFullScreen();
        }

        customizeSubs.getCustomizeEvent(`full:screen:${panelId}`).publish(false);

        if (fullScreenHookConfig?.afterExitFullScreen) {
            fullScreenHookConfig?.afterExitFullScreen();
        }
    }, [customizeSubs, panelId]);

    return [enterFullScreen, exitFullScreen];
}
