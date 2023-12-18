import React, { useEffect, useState } from 'react';
import { usePNCMonitorContext } from '@dreamview/dreamview-core/src/components/panels/PncMonitor/PNCMonitorContext';
import ChartBase, { initOptions } from '@dreamview/dreamview-core/src/components/panels/PncMonitor/Chart/ChartBase';

function CurvatureGraph() {
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
                scale: true,
                dataset,
                series,
                xAxis: {
                    type: 'value',
                    name: 'Time (s)',
                },
                yAxis: {
                    name: 'Curvature (1/m)',
                },
            }),
        );
    };

    useEffect(() => {
        triggerUIUpdate(controlData.curvature);
    }, [controlData]);

    return (
        <ChartBase
            onRefresh={() => onClearControlView(['curvature'])}
            labelRotateBoundary={550}
            title='Curvature'
            options={options}
        />
    );
}
export default React.memo(CurvatureGraph);
