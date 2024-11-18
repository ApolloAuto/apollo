// 判断系统类型
export const isMac = (): boolean => /macintosh|mac os x/i.test(navigator.userAgent);
export const isWin = (): boolean => {
    const agent = navigator.userAgent.toLowerCase();
    return agent.indexOf('win32') >= 0 || agent.indexOf('win64') >= 0;
};

export const getLocalVizPointCloudChannelName = (panelId: string) => `${panelId}-viz-pointcloud-channel`;
export const getLocalVizCurbPointCloudChannelName = (panelId: string) => `${panelId}-viz-curbpointcloud-channel`;
