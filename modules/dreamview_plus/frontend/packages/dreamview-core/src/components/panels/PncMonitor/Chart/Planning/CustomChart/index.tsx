/* eslint-disable prettier/prettier */
import React, { useEffect, useState } from 'react';
import { usePNCMonitorContext } from '@dreamview/dreamview-core/src/components/panels/PncMonitor/PNCMonitorContext';
import ChartBase, { initOptions } from '@dreamview/dreamview-core/src/components/panels/PncMonitor/Chart/ChartBase';
import { hmiUtils } from '@dreamview/dreamview-core/src/store/HmiStore';

function Space() {
    return <div style={{ height: '12px' }} />;
}

function AccelerationGraph() {
    const [options, setOptions] = useState<any>([]);
    const { planningData } = usePNCMonitorContext();

    const triggerUIUpdate = (graphs: any, vehicle: any) => {
        const result = graphs.map((graph: any) => {
            const { options: graphOpt, line, polygon, car } = graph;
            const xAxis = { type: 'value',
                name: graphOpt.x.labelString,
                max: graph.title === 'Open Space Partitioned Trajectory' ? (value: any) => value.max + 5 : graphOpt.x.max,
                min: graph.title === 'Open Space Partitioned Trajectory' ? (value: any) => value.min - 5 : graphOpt.x.min,

            };
            const yAxis = { name: graphOpt.y.labelString,
                max: graph.title === 'Open Space Partitioned Trajectory' ? (value: any) => value.max + 5 : graphOpt.y.max,
                min: graph.title === 'Open Space Partitioned Trajectory' ? (value: any) => value.min - 5 : graphOpt.y.min,
            };
            const dataset = (line || [])
                .filter((subLine: any) => !!subLine.point)
                .map((subLine: any, index: number) => ({
                    id: index,
                    source: [['x', 'y'], ...subLine.point.map((item: any) => [item.x, item.y])],
                }));

            const series = (line || [])
                .filter((subLine: any) => !!subLine.point)
                .map((subLine: any, index: number) => ({
                    datasetId: index,
                    smooth: true,
                    name: subLine.label,
                    type: graph.title === 'Planning S-T Graph' ? 'scatter' : 'line',
                    showSymbol: false,
                    encode: {
                        x: 'x',
                        y: 'y',
                    },
                }));
            const graphic = (polygon || [])
                .filter((subPolygon: any) => !!subPolygon.point)
                .reduce((r: any, subPolygon: any) => {
                    const rect = {
                        type: 'polygon',
                        style: {
                            fill: 'transparent', // 设置多边形颜色
                            stroke: (subPolygon.properties.color || '').replace(/"/g, '') || 'red',
                            lineWidth: subPolygon.properties.borderWidth,
                        },
                        shape: {
                            points: subPolygon.point.map((item: any) => [item.x, item.y]),
                        },
                    };
                    const text = subPolygon.label
                        ? {
                            type: 'text',
                            x: subPolygon.point[2].x,
                            y: subPolygon.point[2].y,
                            style: {
                                text: subPolygon.label,
                                fill: 'red',
                                textAlign: 'right',
                                textVerticalAlign: 'bottom',
                            },
                        }
                        : undefined;
                    if (text) {
                        return [...r, rect, text];
                    }
                    return [...r, rect];
                }, []);

            const graphicCar = (car || []).map((adc: any) => (
                {
                    type: 'polygon',
                    style: {
                        fill: 'transparent', // 设置多边形颜色
                        stroke: adc.color || 'red',
                        lineWidth: 1,
                    },
                    shape: {
                        points: hmiUtils.calculateCarPolygonPoints(
                            adc.x,
                            adc.y,
                            adc.heading,
                            vehicle,
                        ),
                    },
                }
            ));
            graphic.push(...graphicCar);

            return {
                title: graph.title,
                opts: initOptions({
                    xAxis,
                    yAxis,
                    yAxisFormatter(value: any) {
                        if (!value) {
                            return value;
                        }
                        if (typeof value === 'number') {
                            return Number(value.toFixed(0)).toString();
                        }
                        return value;
                    },
                    xAxisFormatter(value: any) {
                        if (!value) {
                            return value;
                        }
                        if (typeof value === 'number') {
                            return Number(value.toFixed(2)).toString();
                        }
                        return value;
                    },
                    scale: true,
                    dataset,
                    series,
                    graphic,
                }),
            };
        });
        setOptions(result);
    };

    useEffect(() => {
        triggerUIUpdate(planningData.customChart, planningData.vehicle);
    }, [planningData]);

    return (
        <>
            {options.map((option: any, index: number) => (
                <>
                    <ChartBase autoHeight key={`${option.title}_${index + 1}`} options={option.opts} title={option.title} />
                    <Space />
                </>
            ))}
        </>
    );
}
export default React.memo(AccelerationGraph);
