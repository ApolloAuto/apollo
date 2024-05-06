/* eslint-disable no-restricted-globals */
export const Global = ((): any => {
    // 浏览器环境
    if (typeof window !== 'undefined') {
        return window;
    }
    // Web Worker环境
    if (typeof self !== 'undefined') {
        return self;
    }
    // Node.js环境
    if (typeof global !== 'undefined') {
        return global;
    }
    // 默认返回一个空对象，防止出现引用错误
    return {};
})();

export const isDarkMode = (): boolean => Global.matchMedia && Global.matchMedia('(prefers-color-scheme: dark)').matches;
