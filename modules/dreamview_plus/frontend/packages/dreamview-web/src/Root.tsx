import { App } from '@dreamview/dreamview-core';
import { useEffect } from 'react';
import Logger from '@dreamview/log';

const logger = Logger.getInstance(__filename);

export default function Root() {
    // fixme: 本地无法使用pwa接口，暂时注释
    // useEffect(() => {
    //     if ('serviceWorker' in navigator) {
    //         navigator.serviceWorker
    //             .register('/service-worker.js')
    //             .then((registration) => {
    //                 logger.info('Dreamview service Worker registered with scope:', registration.scope);
    //             })
    //             .catch((error) => {
    //                 console.log('Dreamview service Worker registration failed:', error);
    //             });
    //     }
    // }, []);

    return <App />;
}
