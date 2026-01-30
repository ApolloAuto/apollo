import shortUUID from 'short-uuid';
import get from 'lodash/get';
import { MosaicNode, MosaicDirection, MosaicPath } from 'react-mosaic-component';
import { MosaicDropTargetPosition } from 'react-mosaic-component/lib/internalTypes';

export const getPanelTypeByPanelId = (panelId: string) => panelId.replace(/!.*$/, '');

export const genereatePanelId = (type: string) => {
    const replaceUid = type.replace(/!.*$/, '');
    return `${replaceUid}!${shortUUID.generate()}`;
};

export const genereateNewMosaicNode = (
    layout: MosaicNode<string>,
    path: MosaicPath,
    position: MosaicDropTargetPosition,
    newPanelType: string,
) => {
    const isLeve1 = path.length === 0;

    const originNode = isLeve1 ? layout : get(layout, path);

    let first;
    let second;
    if (position === MosaicDropTargetPosition.TOP || position === MosaicDropTargetPosition.LEFT) {
        first = newPanelType;
        second = originNode;
    } else {
        first = originNode;
        second = newPanelType;
    }

    let direction: MosaicDirection;
    if (position === MosaicDropTargetPosition.TOP || position === MosaicDropTargetPosition.BOTTOM) {
        direction = 'column';
    } else {
        direction = 'row';
    }

    return {
        first,
        second,
        direction,
    };
};
