import React, { useEffect, useRef, useState } from 'react';
import { usePNCMonitorContext } from '@dreamview/dreamview-core/src/components/panels/PncMonitor/PNCMonitorContext';
import ChartBase, { initOptions } from '@dreamview/dreamview-core/src/components/panels/PncMonitor/Chart/ChartBase';

function Speed() {
    const [options, setOptions] = useState<any>();
    const { planningData } = usePNCMonitorContext();

    const triggerUIUpdate = (speedGraph: any) => {
        const { dataset, series } = Object.entries(speedGraph).reduce(
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
                dataset,
                series,
                scale: true,
                xAxis: {
                    type: 'value',
                    name: 'time (s)',
                    max: 10,
                    min: -2,
                },
                xAxisFormatter(value: any) {
                    return value.toString().replace(/\..*/g, '');
                },
                yAxis: {
                    name: 'Speed (m/s)',
                    type: 'value',
                    max: 40,
                    min: -5,
                    interval: 5,
                },
            }),
        );
    };

    useEffect(() => {
        triggerUIUpdate(planningData.speed);
    }, [planningData]);

    return <ChartBase options={options} title='Planning Speed' />;
}
export default React.memo(Speed);
