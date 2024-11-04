const getOnlineWsURL = () => {
    /* eslint-disable no-underscore-dangle */
    const protocolDreamview = window.location.protocol;
    const protocol = protocolDreamview === 'https:' ? 'wss://' : 'ws://';
    const baseURLDreamview = protocol + window.location.host + window.location.pathname;
    let onlineBaseURL = baseURLDreamview;
    const chars = baseURLDreamview.split('');
    if (chars.length > 0 && chars[chars.length - 1] === '/') {
        chars.pop();
        onlineBaseURL = chars.join('');
    }
    return onlineBaseURL;
};

const getOnlineHttpURL = () => {
    /* eslint-disable no-underscore-dangle */
    const protocolDreamview = `${window.location.protocol}//`;
    const baseURLDreamview = protocolDreamview + window.location.host + window.location.pathname;
    let onlineBaseURL = baseURLDreamview;
    const chars = baseURLDreamview.split('');
    if (chars.length > 0 && chars[chars.length - 1] === '/') {
        chars.pop();
        onlineBaseURL = chars.join('');
    }
    return onlineBaseURL;
};

export const baseURL = process.env.NODE_ENV === 'production' ? getOnlineWsURL() : 'ws://localhost:8888';

export const baseHttpURL =
    process.env.NODE_ENV === 'production' ? window.location.origin : `${window.location.protocol}//localhost:8888`;

// 获取协议
export const getProtocol = () => `${window.location.protocol}//`;

// 获取ws协议
export const getWsProtocol = () => (getProtocol() === 'http://' ? 'ws://' : 'wss://');

// 获取host
export const getHost = () => window.location.hostname;
export const config = {
    baseURL,
    baseHttpURL,
    mainUrl: `${baseURL}/websocket`,
    pluginUrl: `${baseURL}/plugin`,
};

export const requestIdleCallbackTimeout = 2000;
