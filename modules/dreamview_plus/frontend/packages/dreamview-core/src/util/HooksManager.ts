import { FullScreenHookConfig, HookResType } from '../components/panels/base/store/PanelStore';

interface IFullScreenHookConfig {
    addHook: (panelId: string, config: FullScreenHookConfig) => void;
    removeHook: (panelId: string) => void;
    updateHook: (panelId: string, config: Partial<FullScreenHookConfig>) => void;
    getHook: (panelId: string) => FullScreenHookConfig;
}

class HooksManager implements IFullScreenHookConfig {
    private fullScreenHooks: Map<string, FullScreenHookConfig> = new Map();

    public addHook(panelId: string, config: FullScreenHookConfig) {
        if (!this.fullScreenHooks.has(panelId)) {
            this.fullScreenHooks.set(panelId, config);
        }
    }

    public removeHook: (panelId: string) => void;

    public updateHook: (panelId: string, config: Partial<FullScreenHookConfig>) => void;

    public getHook(panelId: string): FullScreenHookConfig {
        return this.fullScreenHooks.get(panelId);
    }

    public async handleFullScreenBeforeHook(hook: () => HookResType) {
        const res = hook();
        if (res === null || res === undefined) {
            return true;
        }
        if (res instanceof Boolean) {
            return res as boolean;
        }
        if (res instanceof Promise) {
            return Boolean(await res);
        }
        return Boolean(res);
    }
}

export const hooksManager = new HooksManager();
