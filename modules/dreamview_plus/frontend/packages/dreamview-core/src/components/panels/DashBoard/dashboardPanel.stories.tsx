import React, { useCallback } from 'react';
import { Meta, StoryObj } from '@storybook/react';
import { Mosaic, MosaicNode, MosaicPath } from 'react-mosaic-component';
import { useMosaicId } from '@dreamview/dreamview-core/src/store/PanelLayoutStore';
import RenderTile from '@dreamview/dreamview-core/src/components/panels/base/RenderTile';
import { genereatePanelId } from '@dreamview/dreamview-core/src/util/layout';
import { noop } from '@dreamview/dreamview-core/src/util/similarFunctions';
import { useMakeStyle } from '@dreamview/dreamview-theme';

function useStyle() {
    const hoc = useMakeStyle(() => ({
        'layout-root': {
            width: 500,
            height: 600,
            '& .mosaic-window-body': {
                height: 600,
            },
        },
    }));
    return hoc();
}

const layout: MosaicNode<string> = genereatePanelId('dashBoard');

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
    title: 'Panel/DashBoard',
    component: DreamviewWindow,
};

export const Primary: StoryObj<typeof DreamviewWindow> = {};

export default meta;
