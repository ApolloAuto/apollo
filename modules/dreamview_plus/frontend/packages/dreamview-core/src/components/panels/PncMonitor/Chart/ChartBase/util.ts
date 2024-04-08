import { LinearInterpolant } from 'three';

/* eslint-disable indent */
const scaleOption = {
    dataZoom: [
        {
            type: 'inside',
        },
    ],
};

export const initOptions = ({
    xAxis,
    xAxisFormatter,
    yAxisFormatter,
    yAxis,
    dataset,
    series,
    scale,
    graphic,
    grid,
    animation = false,
}: {
    animation?: boolean;
    dataset: any;
    series: any;
    xAxis: {
        type: string;
        name: string;
        max?: number;
        min?: number;
        nameGap?: number;
        axisLabel?: any;
        axisEqual?: boolean;
    };
    yAxis: {
        type?: string;
        name: string;
        max?: number;
        min?: number;
        interval?: number;
    };
    scale?: boolean;
    xAxisFormatter?: (value: any) => string;
    yAxisFormatter?: (value: any) => string;
    graphic?: any;
    grid?: {
        containLabel?: boolean;
        top?: number;
        left?: number; // 设置图表的左边界
        right?: number; // 设置图表的右边界
        bottom?: number; // 设置图表的下边界
    };
}) => ({
    graphic,
    grid: {
        containLabel: true,
        top: 40,
        left: 15, // 设置图表的左边界
        right: 15, // 设置图表的右边界
        bottom: 30, // 设置图表的下边界
        ...(grid || {}),
    },
    xAxis: {
        ...xAxis,
        // 是否是脱离 0 值比例。设置成 true 后坐标刻度不会强制包含零刻度。在双数值轴的散点图中比较有用。
        scale: true,
        nameGap: xAxis.nameGap || 30,
        nameLocation: 'middle',
        axisLine: {
            onZero: false, // 将 onZero 设置为 false，以确保 X 轴线不与 Y=0 的轴线对齐
            lineStyle: {
                color: '#A6B5CC',
            },
        },
        splitLine: {
            show: false,
        },
        axisTick: {
            show: true,
            lineStyle: {
                color: '#A6B5CC',
            },
        },
        axisLabel: {
            ...(xAxis.axisLabel || {}),
            formatter(value: any) {
                if (xAxisFormatter) {
                    return xAxisFormatter(value);
                }

                return value;
            },
        },
    },
    yAxis: {
        ...yAxis,
        // 是否是脱离 0 值比例。设置成 true 后坐标刻度不会强制包含零刻度。在双数值轴的散点图中比较有用。
        scale: true,
        axisTick: {
            show: false,
        },
        axisLine: {
            show: false,
        },
        splitLine: {
            show: true,
            lineStyle: {
                type: 'dashed',
                color: '#40454D',
            },
        },
        axisLabel: {
            formatter(value: any) {
                if (yAxisFormatter) {
                    return yAxisFormatter(value);
                }

                return value;
            },
        },
    },
    dataset,
    series,
    animation: false,
    // 图例
    legend: {
        show: false,
        zLevel: 2,
    },
    textStyle: {
        color: '#808B9D',
    },
    ...(scale ? scaleOption : {}),
});

export function extractDataPoints(
    data: any,
    xField: string,
    yField: string,
    loopBack = false,
    xOffset = 0,
    filterRepeatValue = true,
) {
    if (!data) {
        return [];
    }

    const memo: Record<string, boolean> = {};
    let points = data.map((point: any) => [point[xField] + xOffset, point[yField]]);

    if (filterRepeatValue) {
        points = points.filter(([time]: any) => {
            const real = time;
            if (memo[real]) {
                return false;
            }
            memo[real] = true;
            return true;
        });
    }

    if (loopBack && data.length) {
        points.push([data[0][xField], data[0][yField]]);
    }

    return points;
}

export function generateDataPoints(X: any, Y: any, transform?: any) {
    if (!X || !Y || X.length !== Y.length) {
        return [];
    }

    const bound = [];
    for (let idx = 0; idx < Y.length; idx += 1) {
        const x = Number(X[idx]);
        let y = Number(Y[idx]);
        if (transform !== undefined) {
            y = transform(y);
        }
        bound.push([x, y]);
    }
    return bound;
}

export function interpolateValueByCurrentTime(trajectory: any, currentTime: any, fieldName: any) {
    if (fieldName === 'timestampSec') {
        return currentTime;
    }
    const absoluteTimes = (trajectory || []).map((point: any) => point.timestampSec);
    const plannedValues = (trajectory || []).map((point: any) => point[fieldName]);
    const interpolant = new LinearInterpolant(absoluteTimes, plannedValues, 1, []);
    return interpolant.evaluate(currentTime)[0];
}
