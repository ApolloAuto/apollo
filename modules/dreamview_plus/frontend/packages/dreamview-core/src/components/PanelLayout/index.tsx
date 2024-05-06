/* eslint-disable react/jsx-no-useless-fragment */
import React, { useCallback } from 'react';
import { Mosaic, MosaicPath, MosaicNode } from 'react-mosaic-component';
import 'react-mosaic-component/react-mosaic-component.css';
import { update } from '@dreamview/dreamview-core/src/store/PanelLayoutStore/actions';
import {
    useGetCurrentLayout,
    usePanelLayoutStore,
    useMosaicId,
} from '@dreamview/dreamview-core/src/store/PanelLayoutStore';
import RenderTile from '@dreamview/dreamview-core/src/components/panels/base/RenderTile';
import { usePickHmiStore } from '@dreamview/dreamview-core/src/store/HmiStore';
import useStyle from './style';
import PanelEmpty from './PanelEmpty';

/**
 * Panel layout component.
 */
function DreamviewWindow() {
    const mosaicId = useMosaicId();
    const [hmi] = usePickHmiStore();
    const [, dispatch] = usePanelLayoutStore();
    const layout = useGetCurrentLayout();

    const onMosaicChange = (newNode: MosaicNode<string>) => {
        dispatch(update({ mode: hmi.currentMode, layout: newNode }));
    };

    const renderTile = useCallback(
        (panelId: string, path: MosaicPath) => <RenderTile panelId={panelId} path={path} />,
        [],
    );

    return (
        <Mosaic<string>
            className='my-mosaic'
            mosaicId={mosaicId}
            renderTile={renderTile}
            value={layout}
            onChange={onMosaicChange}
        />
    );
}

const DreamviewWindowMemo = React.memo(DreamviewWindow);

function PanelLayout() {
    const { classes } = useStyle();
    const layout = useGetCurrentLayout();
    return <div className={classes['layout-root']}>{layout ? <DreamviewWindowMemo /> : <PanelEmpty />}</div>;
}

export default React.memo(PanelLayout);
