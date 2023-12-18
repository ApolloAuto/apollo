import { WebSocketPlugin } from './type';

export class PluginManager {
    private plugins: WebSocketPlugin[] = [];

    registerPlugin(plugin: WebSocketPlugin): void {
        this.plugins.push(plugin);
    }

    getPluginsForInflowDataName(dataName: string): WebSocketPlugin[] {
        return this.plugins.filter((plugin) => plugin.inflowDataName === dataName);
    }

    getPluginsForDataName(dataName: string): WebSocketPlugin[] {
        return this.plugins.filter((plugin) => plugin.dataName === dataName);
    }

    getPluginsForChannel(dataName: string, channelName: string): WebSocketPlugin[] {
        return this.plugins.filter((plugin) => plugin.dataName === dataName && plugin.channelName === channelName);
    }

    getAllPlugins(): WebSocketPlugin[] {
        return this.plugins;
    }
}
