import React, { useEffect, useState } from 'react';
import { usePNCMonitorContext } from '@dreamview/dreamview-core/src/components/panels/PncMonitor/PNCMonitorContext';
import ChartBase, { initOptions } from '../../ChartBase';

function TrajectoryGraph() {
    const [options, setOptions] = useState<any>();
    const { controlData, onClearControlView } = usePNCMonitorContext();

    const getMaxAndMin = (values: any, prevX: { max: number; min: number }, prevy: { max: number; min: number }) => {
        if (!values?.length) {
            return {
                x: prevX,
                y: prevy,
            };
        }
        const x = { ...prevX };
        const y = { ...prevy };
        (values || []).forEach(([xValue, yValue]: [number, number]) => {
            if (xValue && yValue) {
                x.max = Math.max(xValue, x.max);
                x.min = Math.min(xValue, x.min);
                y.max = Math.max(yValue, y.max);
                y.min = Math.min(yValue, y.min);
            }
        });
        return {
            x,
            y,
        };
    };

    const triggerUIUpdate = (graph: any, polygon: any) => {
        const { dataset, series, x, y } = Object.entries(graph).reduce(
            (result, [key, value]) => ({
                dataset: [...result.dataset, { id: key, source: [['x', 'y'], ...(value as any)] }],
                series: [
                    ...result.series,
                    {
                        datasetId: key,
                        smooth: true,
                        name: key,
                        type: 'line',
                        showSymbol: false,
                        encode: {
                            x: 'x',
                            y: 'y',
                        },
                    },
                ],
                ...getMaxAndMin(value, result.x, result.y),
            }),
            {
                dataset: [],
                series: [],
                x: {
                    max: -1,
                    min: Infinity,
                },
                y: {
                    max: -1,
                    min: Infinity,
                },
            },
        );
        const graphic = [
            {
                type: 'polygon',
                style: {
                    fill: 'transparent', // 设置多边形颜色
                    stroke: 'red',
                },
                shape: {
                    points: polygon,
                },
            },
        ];

        const xmid = Math.ceil((x.min + x.max) / 2);
        const ymid = Math.ceil((y.min + y.max) / 2);
        const gap = Math.ceil(Math.max(x.max - x.min, y.max - y.min) / 2);
        const offset = 10;

        setOptions(
            initOptions({
                animation: true,
                dataset,
                series,
                graphic,
                grid: {
                    containLabel: false,
                    left: 70,
                    top: 35,
                    bottom: 70,
                    right: 35,
                },
                xAxis: {
                    type: 'value',
                    name: 'x(m)',
                    axisEqual: true,
                    max: xmid + gap + offset,
                    min: xmid - gap - offset,
                    nameGap: 70,
                    axisLabel: {
                        rotate: 45,
                    },
                },
                yAxis: {
                    type: 'value',
                    name: 'y(m)',
                    max: ymid + gap + offset,
                    min: ymid - gap - offset,
                },
                xAxisFormatter(value: any) {
                    return value.toString();
                },
                yAxisFormatter(value: any) {
                    return value.toString();
                },
            }),
        );
    };

    useEffect(() => {
        triggerUIUpdate(controlData.trajectory, controlData.polygon);
    }, [controlData]);

    return (
        <ChartBase
            onRefresh={() => onClearControlView(['trajectory', 'polygon'])}
            autoHeight
            title='TrajectoryGraph'
            options={options}
        />
    );
}
export default React.memo(TrajectoryGraph);
