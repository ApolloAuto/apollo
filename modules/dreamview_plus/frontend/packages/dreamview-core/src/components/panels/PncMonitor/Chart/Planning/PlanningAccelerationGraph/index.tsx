import React, { useEffect, useState } from 'react';
import { usePNCMonitorContext } from '@dreamview/dreamview-core/src/components/panels/PncMonitor/PNCMonitorContext';
import ChartBase, { initOptions } from '@dreamview/dreamview-core/src/components/panels/PncMonitor/Chart/ChartBase';

function AccelerationGraph() {
    const [options, setOptions] = useState<any>();
    const { planningData } = usePNCMonitorContext();

    const triggerUIUpdate = (graph: any) => {
        const { dataset, series } = Object.entries(graph).reduce(
            (result, [key, value]: any) => ({
                dataset: [
                    ...result.dataset,
                    { id: key, source: [['x', 'y'], ...value].filter(([x]: any) => x === 'x' || x > -2) },
                ],
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
            }),
            {
                dataset: [],
                series: [],
            },
        );

        setOptions(
            initOptions({
                // scale: true,
                dataset,
                series,
                xAxis: { type: 'value', name: 'Time (s)', max: 10, min: -2 },
                yAxis: { interval: 1, name: 'Acceleration (m/s^2)', min: -4, max: 4 },
            }),
        );
    };

    useEffect(() => {
        triggerUIUpdate(planningData.acceleration);
    }, [planningData]);

    return <ChartBase options={options} title='Planning Acceleration' />;
}
export default React.memo(AccelerationGraph);
