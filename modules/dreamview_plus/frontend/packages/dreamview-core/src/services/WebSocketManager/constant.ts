const getOnlineBaseURL = () => {
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

const baseURL = process.env.NODE_ENV === 'production' ? getOnlineBaseURL() : 'ws://localhost:8888';

export const config = {
    baseURL,
    mainUrl: `${baseURL}/websocket`,
    pluginUrl: `${baseURL}/plugin`,
};
