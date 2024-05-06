import React, { useEffect, useState } from 'react';
import { usePNCMonitorContext } from '@dreamview/dreamview-core/src/components/panels/PncMonitor/PNCMonitorContext';
import ChartBase, { initOptions } from '@dreamview/dreamview-core/src/components/panels/PncMonitor/Chart/ChartBase';

function StationErrorGraph() {
    const [options, setOptions] = useState<any>(null);
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
                        name: 'stationError',
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
                xAxis: {
                    type: 'value',
                    name: 'Time (s)',
                },
                yAxis: {
                    name: 'Error (m)',
                },
                scale: true,
            }),
        );
    };

    useEffect(() => {
        triggerUIUpdate(controlData.stationError);
    }, [controlData]);

    return (
        <ChartBase
            onRefresh={() => onClearControlView(['stationError'])}
            labelRotateBoundary={550}
            title='Station Error'
            options={options}
        />
    );
}
export default React.memo(StationErrorGraph);
