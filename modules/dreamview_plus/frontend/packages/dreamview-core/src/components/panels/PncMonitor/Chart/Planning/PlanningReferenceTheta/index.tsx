import React, { useEffect, useState } from 'react';
import { usePNCMonitorContext } from '@dreamview/dreamview-core/src/components/panels/PncMonitor/PNCMonitorContext';
import ChartBase, { initOptions } from '@dreamview/dreamview-core/src/components/panels/PncMonitor/Chart/ChartBase';

function Theta() {
    const [options, setOptions] = useState<any>();
    const { planningData } = usePNCMonitorContext();

    const triggerUIUpdate = (graph: any) => {
        const { dataset, series } = Object.entries(graph)
            .filter(([id]: any) => id === 'ReferenceLine')
            .reduce(
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
                    name: 'S (m)',
                },
                yAxis: {
                    name: 'Theta',
                },
            }),
        );
    };

    useEffect(() => {
        triggerUIUpdate(planningData.referenceTheta);
    }, [planningData]);

    return <ChartBase title='Reference Line Theta' options={options} />;
}
export default React.memo(Theta);
