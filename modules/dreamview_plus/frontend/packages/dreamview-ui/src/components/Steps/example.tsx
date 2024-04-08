import React, { useState } from 'react';
import { StepsProps } from 'antd';
import MapSerialNumberOne from '@dreamview/dreamview-core/src/assets/svg/mapCollect/map_serial_number_one';
import MapSerialNumberTwo from '@dreamview/dreamview-core/src/assets/svg/mapCollect/map_serial_number_two';
import MapSerialNumberThree from '@dreamview/dreamview-core/src/assets/svg/mapCollect/map_serial_number_three';
import MapSerialNumberNotOne from '@dreamview/dreamview-core/src/assets/svg/mapCollect/map_serial_number_not_one';
import MapSerialNumberNotTwo from '@dreamview/dreamview-core/src/assets/svg/mapCollect/map_serial_number_not_two';
import MapSerialNumberNotThree from '@dreamview/dreamview-core/src/assets/svg/mapCollect/map_serial_number_not_three';
import MapSerialNumberFinish from '@dreamview/dreamview-core/src/assets/svg/mapCollect/map_serial_number_finish';
import { Steps } from '.';

export function Template(props: StepsProps) {
    const [currentSteps, setCurrentSteps] = useState(1);
    const collectSteps = [
        {
            title: '环境检测',
            icon: (() => {
                if (currentSteps === 0) {
                    return <MapSerialNumberOne />;
                }
                if (currentSteps > 0) {
                    return <MapSerialNumberFinish />;
                }
            })(),
        },
        {
            title: '数据采集',
            icon: (() => {
                if (currentSteps < 1) {
                    return <MapSerialNumberNotTwo />;
                }
                if (currentSteps === 1) {
                    return <MapSerialNumberTwo />;
                }
                if (currentSteps > 1) {
                    return <MapSerialNumberFinish />;
                }
            })(),
        },
        {
            title: '文件导出',
            icon: (() => {
                if (currentSteps < 2) {
                    return <MapSerialNumberNotThree />;
                }
                if (currentSteps === 2) {
                    return <MapSerialNumberThree />;
                }
                if (currentSteps > 2) {
                    return <MapSerialNumberFinish />;
                }
            })(),
        },
    ];

    return (
        <div>
            <Steps current={1} items={collectSteps} />
        </div>
    );
}
