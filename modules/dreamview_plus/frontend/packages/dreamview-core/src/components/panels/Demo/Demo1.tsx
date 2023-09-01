import React, { useEffect } from 'react';
import { usePanelInnerStore } from '@dreamview/dreamview-core/src/store/PanelInnerStore';
import Panel from '@dreamview/dreamview-core/src/components/panels/base/Panel';
import { IRenderToolBar } from '../type/RenderToolBar';

export function RenderToolBar(props: IRenderToolBar) {
    const [{ ee }] = usePanelInnerStore();

    return <div onClick={() => ee.emit('test', 'testvalue')}>demo1 toolbar</div>;
}

function DemoPanel() {
    const [{ ee }] = usePanelInnerStore();
    useEffect(() => {
        ee.addListener('test', (val: any) => {
            console.log('addEventListenerval', val);
        });
        ee.on('test', (val: any) => {
            console.log('onval', val);
        });
    }, []);

    return <div>demo1</div>;
}

export default Panel(DemoPanel);
