// 判断系统类型
export const isMac = (): boolean => /macintosh|mac os x/i.test(navigator.userAgent);
export const isWin = (): boolean => {
    const agent = navigator.userAgent.toLowerCase();
    return agent.indexOf('win32') >= 0 || agent.indexOf('win64') >= 0;
};
