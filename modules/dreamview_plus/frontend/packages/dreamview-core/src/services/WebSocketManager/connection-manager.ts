import Logger from '@dreamview/log';
import { WebSocketConnection } from './websocket-connect.service';

const logger = Logger.getInstance('ConnectionManager');

export class ConnectionManager {
    webSockets: Map<string, WebSocketConnection> = new Map();

    constructor() {
        logger.info('ConnectionManager created');
    }

    connect(url: string, retries = 3, retryInterval = 1000) {
        if (!this.has(url)) {
            const webSocketConnection = new WebSocketConnection(url);
            this.set(url, webSocketConnection);
            return webSocketConnection.connect(retries, retryInterval);
        }
        return this.get(url)?.connectionStatus$;
    }

    set(url: string, webSocketConnection: WebSocketConnection) {
        this.webSockets.set(url, webSocketConnection);
    }

    get(url: string) {
        if (!this.has(url)) {
            this.connect(url);
        }
        return this.webSockets.get(url);
    }

    // 单纯获取连接状态
    getConnection(url: string) {
        return this.webSockets.get(url);
    }

    has(url: string) {
        return this.webSockets.has(url);
    }

    delete(url: string) {
        const webSocketConnection = this.getConnection(url);
        if (webSocketConnection) {
            webSocketConnection.disconnect();
            this.webSockets.delete(url);
        }
    }

    clear() {
        this.webSockets.clear();
    }

    get size() {
        return this.webSockets.size;
    }
}
