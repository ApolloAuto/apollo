import React, { useEffect, useState } from 'react';
import { usePNCMonitorContext } from '@dreamview/dreamview-core/src/components/panels/PncMonitor/PNCMonitorContext';
import ChartBase, { initOptions } from '@dreamview/dreamview-core/src/components/panels/PncMonitor/Chart/ChartBase';

function LateralErrorGraph() {
    const [options, setOptions] = useState<any>();
    const { controlData, onClearControlView } = usePNCMonitorContext();

    const triggerUIUpdate = (graph: any) => {
        if (!graph?.length) {
            return;
        }
        setOptions(
            initOptions({
                dataset: {
                    source: [['x', 'y'], ...graph],
                },
                series: [
                    {
                        name: 'lateralError',
                        smooth: true,
                        type: 'line',
                        showSymbol: false,
                        itemStyle: {
                            color: '#3288FA',
                        },
                        encode: {
                            x: 'x',
                            y: 'y',
                        },
                    },
                ],
                yAxis: {
                    name: 'Error (m)',
                },
                xAxis: {
                    type: 'value',
                    name: 'Time (s)',
                },
                scale: true,
            }),
        );
    };

    useEffect(() => {
        triggerUIUpdate(controlData.lateralError);
    }, [controlData]);

    return (
        <ChartBase
            onRefresh={() => onClearControlView(['lateralError'])}
            labelRotateBoundary={550}
            title='Lateral Error'
            options={options}
        />
    );
}
export default React.memo(LateralErrorGraph);
