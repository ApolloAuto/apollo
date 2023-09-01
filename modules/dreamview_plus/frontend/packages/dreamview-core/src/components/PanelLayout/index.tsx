/* eslint-disable react/jsx-no-useless-fragment */
import React, { useCallback, useEffect, memo } from 'react';
import { Mosaic, MosaicPath, MosaicNode } from 'react-mosaic-component';
import 'react-mosaic-component/react-mosaic-component.css';
import { update } from '@dreamview/dreamview-core/src/store/PanelLayoutStore/actions';
import useStyle from './style';
import { usePanelLayoutStore, useMosaicId } from '../../store/PanelLayoutStore';
import RenderTile from '../panels/base/RenderTile';
import PanelEmpty from './PanelEmpty';

function DreamviewWindow() {
    const mosaicId = useMosaicId();
    const [{ layout }, dispatch] = usePanelLayoutStore();

    const onMosaicChange = (newNode: MosaicNode<string>) => {
        dispatch(update(newNode));
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
    const [{ layout }] = usePanelLayoutStore();
    return <div className={classes['layout-root']}>{layout ? <DreamviewWindowMemo /> : <PanelEmpty />}</div>;
}

export default React.memo(PanelLayout);
