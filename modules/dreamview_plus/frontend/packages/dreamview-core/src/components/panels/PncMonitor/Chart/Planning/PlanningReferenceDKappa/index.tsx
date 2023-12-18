import React, { useEffect, useState } from 'react';
import { usePNCMonitorContext } from '@dreamview/dreamview-core/src/components/panels/PncMonitor/PNCMonitorContext';
import { fill0 } from '@dreamview/dreamview-core/src/util/misc';
import ChartBase, { initOptions } from '@dreamview/dreamview-core/src/components/panels/PncMonitor/Chart/ChartBase';

function DKappa() {
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
                scale: true,
                series,
                xAxis: {
                    type: 'value',
                    name: 'Time (s)',
                },
                yAxis: {
                    name: 'DKappa',
                    max: 0.02,
                    min: -0.02,
                    interval: 0.005,
                },
                yAxisFormatter(value) {
                    if (Number(value) === 0) {
                        return '0';
                    }
                    const symbol = Number(value) > 0 ? '' : '-';
                    return `${symbol}0.${fill0(Math.abs(value) * 1000, 3)}`;
                },
            }),
        );
    };

    useEffect(() => {
        triggerUIUpdate(planningData.referenceDKappa);
    }, [planningData]);
    return <ChartBase title='Reference Line Kappa Derivative' options={options} />;
}
export default React.memo(DKappa);
