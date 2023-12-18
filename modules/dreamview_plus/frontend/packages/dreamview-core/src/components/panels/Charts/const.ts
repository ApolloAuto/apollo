import shortUUID from 'short-uuid';

export enum CACHE_OPTIONS {
    CACHE = 'cache',
    NOCACHE = 'nocache',
}

export enum HIDDEL_OPTION {
    SHOW = 'show',
    HIDE = 'hide',
}

export enum LINE_WIDTH {
    sm = 1,
    md = 2,
    lg = 3,
}

export enum KEY {
    title = 'title',
    cache = 'cache',
    xAxisName = 'xAxisName',
    yAxisName = 'yAxisName',
    lineList = 'lineList',
    lineDataName = 'lineDataName',
    lineChannel = 'lineChannel',
    lineChannelY = 'lineChannelY',
    lineChannelX = 'lineChannelX',
    lineName = 'lineName',
    lineWidth = 'lineWidth',
    lineColor = 'lineColor',
    lineHidden = 'lineHidden',
}

export interface ILineConfig {
    lineDataName: string;
    lineChannel: string;
    lineChannelY: string;
    lineChannelX: string;
    lineName: string;
    lineWidth: LINE_WIDTH;
    lineColor: string;
    lineHidden: HIDDEL_OPTION;
    uid: string;
    // cache: CACHE_OPTIONS;
}

export interface IChartConfig {
    title: string;
    xAxisName: string;
    yAxisName: string;
    lineList: Array<ILineConfig>;
}

export interface IChartListItem {
    uid: string;
    value: IChartConfig;
}

export const initLineValue: () => ILineConfig = () => ({
    lineDataName: undefined,
    lineChannelY: undefined,
    lineChannelX: undefined,
    lineChannel: undefined,
    lineName: undefined,
    lineWidth: LINE_WIDTH.sm,
    lineColor: '#3288fa',
    lineHidden: HIDDEL_OPTION.SHOW,
    uid: shortUUID.generate(),
    // cache: CACHE_OPTIONS.NOCACHE,
});

export const initChartValue: () => IChartConfig = () => ({
    title: undefined,
    xAxisName: undefined,
    yAxisName: undefined,
    lineList: [initLineValue()],
});

export interface IChannelListItem {
    label: string;
    value: string;
    dataName: string;
    channelName: string;
    msgType: string;
    protoPath: string;
}

export type IChannelList = IChannelListItem[];
