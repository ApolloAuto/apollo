import React, { useEffect, useState } from 'react';
import { usePNCMonitorContext } from '@dreamview/dreamview-core/src/components/panels/PncMonitor/PNCMonitorContext';
import ChartBase, { initOptions } from '@dreamview/dreamview-core/src/components/panels/PncMonitor/Chart/ChartBase';

function Latency() {
    const [options, setOptions] = useState<any>();
    const { latencyData, onClearLatencyView } = usePNCMonitorContext();

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
                    name: 'Latency (ms)',
                },
                xAxisFormatter(value) {
                    return value.toString().replace(/\..*/g, '');
                },
            }),
        );
    };

    useEffect(() => {
        triggerUIUpdate(latencyData);
    }, [latencyData]);

    return <ChartBase onRefresh={onClearLatencyView} labelRotateBoundary={550} title='Latency' options={options} />;
}
export default React.memo(Latency);
