import { AxiosResponse } from 'axios';
import { termAxios } from './axiosUtil';

export type TSize = {
    rows: number;
    cols: number;
};

/**
 * 获取PID
 * @param size 尺寸
 */
export const getPid = (size: TSize): Promise<AxiosResponse<string>> =>
    termAxios.post(`/terminals?cols=${size.cols}&rows=${size.rows}`);

export const adjustTerminalSize = (pid: string, size: TSize): Promise<unknown> =>
    termAxios.post(`/terminals/${pid}/size?cols=${size.cols}&rows=${size.rows}`);
