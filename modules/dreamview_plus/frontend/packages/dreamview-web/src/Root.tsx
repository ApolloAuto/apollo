import { App } from '@dreamview/dreamview-core';
import { useLayoutEffect } from 'react';
import Logger from '@dreamview/log';
import * as process from 'process';

const logger = Logger.getInstance(__filename);

export default function Root() {
    // todo： 暂时去除service worker注册
    // useLayoutEffect(() => {
    //     if (process.env.NODE_ENV === 'production') {
    //         logger.info('dreamview-web init');
    //         if ('serviceWorker' in navigator) {
    //             navigator.serviceWorker
    //                 .register('/service-worker.js')
    //                 .then((registration) => {
    //                     logger.info('Dreamview service Worker registered with scope:', registration.scope);
    //                 })
    //                 .catch((error) => {
    //                     logger.error('Dreamview service Worker registration failed:', error);
    //                 });
    //         }
    //     }
    // }, []);

    return <App />;
}
