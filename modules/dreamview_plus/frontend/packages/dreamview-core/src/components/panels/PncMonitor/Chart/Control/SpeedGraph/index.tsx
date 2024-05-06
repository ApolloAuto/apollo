import React, { useEffect, useState } from 'react';
import { usePNCMonitorContext } from '@dreamview/dreamview-core/src/components/panels/PncMonitor/PNCMonitorContext';
import ChartBase, { initOptions } from '@dreamview/dreamview-core/src/components/panels/PncMonitor/Chart/ChartBase';

function SpeedGraph() {
    const [options, setOptions] = useState<any>();
    const { controlData, onClearControlView } = usePNCMonitorContext();

    const triggerUIUpdate = (graph: any) => {
        const { dataset, series } = Object.entries(graph).reduce(
            (result, [key, value]: any) => ({
                dataset: [...result.dataset, { id: key, source: [['x', 'y'], ...value] }],
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
                xAxis: {
                    type: 'value',
                    name: 'Time (s)',
                },
                yAxis: {
                    name: 'Speed(m/s)',
                },
                scale: true,
            }),
        );
    };

    useEffect(() => {
        triggerUIUpdate(controlData.speed);
    }, [controlData]);

    return (
        <ChartBase
            onRefresh={() => onClearControlView(['speed'])}
            labelRotateBoundary={550}
            title='Speed'
            options={options}
        />
    );
}
export default React.memo(SpeedGraph);
