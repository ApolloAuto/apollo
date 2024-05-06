import { MosaicDragType, MosaicPath } from 'react-mosaic-component';
import { ConnectDragPreview, useDrag, ConnectDragSource } from 'react-dnd';
import { usePanelLayoutStore } from '@dreamview/dreamview-core/src/store/PanelLayoutStore';
import { IPanelMetaInfo } from '@dreamview/dreamview-core/src/components/panels/type/Panel';
import { addPanelFromOutside } from '@dreamview/dreamview-core/src/store/PanelLayoutStore/actions';
import { usePickHmiStore } from '@dreamview/dreamview-core/src/store/HmiStore';

interface IDropResult {
    path: MosaicPath;
    position: 'top' | 'bottom' | 'left' | 'right';
}

export default function useDragPanel(mosaicId: string, panel: IPanelMetaInfo): [ConnectDragSource, ConnectDragPreview] {
    const [hmi] = usePickHmiStore();
    const [, dispatch] = usePanelLayoutStore();
    const [, connectDragSource, connectDragPreview] = useDrag(() => ({
        type: MosaicDragType.WINDOW,
        item: () => ({ mosaicId, panelConfig: panel }),
        options: { dropEffect: 'copy' },
        end(_, monitor) {
            const dropResult = monitor.getDropResult() as IDropResult;
            // 拖拽的目标不是有效拖拽目标时dropResult为空
            if (!dropResult) return;
            const { position, path: destinationPath } = dropResult;
            dispatch(
                addPanelFromOutside({
                    mode: hmi.currentMode,
                    path: destinationPath,
                    position,
                    originPanelConfig: panel,
                }),
            );
        },
    }));

    return [connectDragSource, connectDragPreview];
}
