import React, { useCallback } from 'react';
import { Meta, StoryObj } from '@storybook/react';
import { Mosaic, MosaicNode, MosaicPath } from 'react-mosaic-component';
import { useMosaicId } from '@dreamview/dreamview-core/src/store/PanelLayoutStore';
import RenderTile from '@dreamview/dreamview-core/src/components/panels/base/RenderTile';
import { genereatePanelId } from '@dreamview/dreamview-core/src/util/layout';
import { noop } from '@dreamview/dreamview-core/src/util/similarFunctions';
import { makeStyles } from '@dreamview/dreamview-theme';

const useStyle = makeStyles(() => ({
    'layout-root': {
        width: 500,
        height: 600,
        '& .mosaic-window-body': {
            height: 600,
        },
    },
}));

const layout: MosaicNode<string> = genereatePanelId('terminalWin');

function DreamviewWindow() {
    const { classes } = useStyle();

    const mosaicId = useMosaicId();

    const renderTile = useCallback(
        (panelId: string, path: MosaicPath) => <RenderTile panelId={panelId} path={path} />,
        [],
    );

    return (
        <div className={classes['layout-root']}>
            <Mosaic<string>
                className='my-mosaic'
                mosaicId={mosaicId}
                renderTile={renderTile}
                value={layout}
                onChange={noop}
            />
        </div>
    );
}

const meta: Meta<typeof DreamviewWindow> = {
    title: 'Panel/TerminalWin',
    component: DreamviewWindow,
};

export const Primary: StoryObj<typeof DreamviewWindow> = {};

export default meta;
