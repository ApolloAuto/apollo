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

export const config = {
    baseURL,
    baseHttpURL,
    mainUrl: `${baseURL}/websocket`,
    pluginUrl: `${baseURL}/plugin`,
};

export const requestIdleCallbackTimeout = 2000;
