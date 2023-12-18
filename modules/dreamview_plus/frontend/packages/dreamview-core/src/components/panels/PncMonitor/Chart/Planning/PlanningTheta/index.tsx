import React, { useEffect, useState } from 'react';
import { usePNCMonitorContext } from '@dreamview/dreamview-core/src/components/panels/PncMonitor/PNCMonitorContext';
import ChartBase, { initOptions } from '@dreamview/dreamview-core/src/components/panels/PncMonitor/Chart/ChartBase';

function Theta() {
    const [options, setOptions] = useState<any>();
    const { planningData } = usePNCMonitorContext();

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
                scale: true,
                xAxis: {
                    type: 'value',
                    name: 'Time (s)',
                },
                yAxis: {
                    name: 'Theta',
                },
                xAxisFormatter(value: any) {
                    return value.toString().replace(/\..*/g, '');
                },
            }),
        );
    };

    useEffect(() => {
        triggerUIUpdate(planningData.theta);
    }, [planningData]);

    return <ChartBase title='Planning Theta' options={options} labelRotateBoundary={550} />;
}
export default React.memo(Theta);
