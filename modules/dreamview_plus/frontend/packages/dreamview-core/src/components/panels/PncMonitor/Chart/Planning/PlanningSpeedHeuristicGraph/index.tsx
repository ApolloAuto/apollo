import React, { useEffect, useState } from 'react';
import { usePNCMonitorContext } from '@dreamview/dreamview-core/src/components/panels/PncMonitor/PNCMonitorContext';
import ChartBase, { initOptions } from '@dreamview/dreamview-core/src/components/panels/PncMonitor/Chart/ChartBase';

function SpeedHeuristic() {
    const [options, setOptions] = useState<any>();
    const { planningData } = usePNCMonitorContext();

    const triggerUIUpdate = (graph: any) => {
        const dataset = Object.entries(graph || {})
            .filter(([key]) => key !== 'obstaclesBoundary')
            .map(([id, value]: any) => ({
                id,
                source: [['x', 'y'], ...(value || [])],
            }));
        const series = Object.entries(graph || {})
            .filter(([key]) => key !== 'obstaclesBoundary')
            .map(([id]: any) => ({
                datasetId: id,
                smooth: true,
                name: id,
                type: 'line',
                showSymbol: false,
                encode: {
                    x: 'x',
                    y: 'y',
                },
            }));
        const graphic = Object.entries(graph?.obstaclesBoundary || {}).map(([, points]: any) => ({
            type: 'polygon',
            style: {
                fill: 'transparent', // 设置多边形颜色
                stroke: 'red',
            },
            shape: {
                points,
            },
        }));
        setOptions(
            initOptions({
                dataset,
                series,
                scale: true,
                graphic,
                xAxis: {
                    type: 'value',
                    name: 'Time (s)',
                    min: -2,
                    max: 10,
                },
                yAxis: {
                    name: 'S (m)',
                    max: 220,
                    min: -10,
                },
            }),
        );
    };

    useEffect(() => {
        triggerUIUpdate(planningData.speedHeuristic.DP_ST_SPEED_OPTIMIZER);
    }, [planningData]);

    return <ChartBase title='Speed Heuristic' options={options} />;
}
export default React.memo(SpeedHeuristic);
