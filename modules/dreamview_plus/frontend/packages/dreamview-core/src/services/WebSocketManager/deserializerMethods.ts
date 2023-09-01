import { WebSocketSubjectConfig } from 'rxjs/internal/observable/dom/WebSocketSubject';
import Logger from '@dreamview/log';

const logger = Logger.getInstance('WebSocketManager');

export const ChildWebSocketSubjectConfig = (url: string) =>
    ({
        url,
        binaryType: 'arraybuffer',
        deserializer: (e) => e.data,
        openObserver: {
            next: () => {
                logger.debug('child websocket connected');
            },
        },
        closeObserver: {
            next: (closeEvent) => {
                logger.debug('child websocket disconnected');
                if (closeEvent.wasClean) {
                    logger.debug('child websocket closed gracefully');
                } else {
                    logger.debug(
                        `child websocket closed unexpectedly, error code: ${closeEvent.code}, error reason: ${closeEvent.reason}`,
                    );
                }
            },
        },
    } as WebSocketSubjectConfig<any>);
