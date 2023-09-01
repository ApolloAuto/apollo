import * as process from 'process';

declare interface Window {
    SANDBOX_PATHNAME: string;
    DREAMVIEW_VERSION: string;
    __REDUX_DEVTOOLS_EXTENSION__: any;
    self: Window;
}

declare module '*.png' {
    const src: string;
    export default src;
}

// eslint-disable-next-line no-underscore-dangle
declare let __PLATFORM__: 'web' | 'desktop';

declare interface process {
    env: {
        NODE_ENV: 'development' | 'production';
    };
}

declare module 'worker-loader!*' {
    const content: any;
    export = content;
}
