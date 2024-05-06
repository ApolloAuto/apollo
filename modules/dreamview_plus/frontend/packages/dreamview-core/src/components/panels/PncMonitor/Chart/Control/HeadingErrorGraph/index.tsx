import React, { useEffect, useState } from 'react';
import { usePNCMonitorContext } from '@dreamview/dreamview-core/src/components/panels/PncMonitor/PNCMonitorContext';
import ChartBase, { initOptions } from '@dreamview/dreamview-core/src/components/panels/PncMonitor/Chart/ChartBase';

function HeadingErrorGraph() {
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
                        smooth: true,
                        type: 'line',
                        showSymbol: false,
                        name: 'headingError',
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
        triggerUIUpdate(controlData.headingError);
    }, [controlData]);

    return (
        <ChartBase
            onRefresh={() => onClearControlView(['headingError'])}
            labelRotateBoundary={550}
            title='Heading Error'
            options={options}
        />
    );
}
export default React.memo(HeadingErrorGraph);
