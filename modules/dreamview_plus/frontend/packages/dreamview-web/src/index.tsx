import React from 'react';
import { createRoot } from 'react-dom/client';
import Logger from '@dreamview/log';
import { initI18n } from '@dreamview/dreamview-lang';
import { PerformanceMonitorPluginAdaptor } from '@dreamview/dreamview-analysis';

const logger = Logger.getInstance(__filename);

export default async function dreamviewWebInit() {
    const rootEl = document.getElementById('root');
    if (!rootEl) {
        throw new Error('missing #root element');
    }

    // fixme： 暂时去除浏览器限制
    // const chromeMatch = navigator.userAgent.match(/Chrome\/(\d+)\./);
    // const chromeVersion = chromeMatch ? parseInt(chromeMatch[1] ?? '', 10) : 0;
    // const isChrome = chromeVersion !== 0;
    //
    // if (!isChrome) {
    //     const message = 'Dreamview is only supported on Chrome.';
    //     const errorEl = document.createElement('div');
    //     errorEl.style.color = 'red';
    //     errorEl.style.fontSize = '2em';
    //     errorEl.style.fontWeight = 'bold';
    //     errorEl.style.margin = '1em';
    //     errorEl.textContent = message;
    //     rootEl.appendChild(errorEl);
    //     throw new Error(message);
    //     return;
    // }

    logger.info('dreamview-web init');

    const { default: Root } = await import('./Root');

    const root = createRoot(rootEl);

    await initI18n();

    root.render(
        <>
            <Root />
            <PerformanceMonitorPluginAdaptor />
        </>
    );
}
