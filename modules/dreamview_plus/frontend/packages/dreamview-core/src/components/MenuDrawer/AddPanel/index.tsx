import React from 'react';
import { usePanelCatalogContext } from '@dreamview/dreamview-core/src/store/PanelCatalogStore';
import { useMosaicId, usePanelLayoutStore } from '@dreamview/dreamview-core/src/store/PanelLayoutStore';
import { IPanelMetaInfo } from '@dreamview/dreamview-core/src/components/panels/type/Panel';
import useDragPanel from '@dreamview/dreamview-core/src/util/useDragPanel';
import { IconIcMove, Popover } from '@dreamview/dreamview-ui';
import { useTranslation } from 'react-i18next';
import MenuDrawerTitle from '../Common/MenuDrawerTitle';
import useStyle from './useStyle';
import { addPanelFromOutside } from '../../../store/PanelLayoutStore/actions';

function PanelThumbnail(props: { panel: IPanelMetaInfo }) {
    const { classes } = useStyle();

    const { title, description, thumbnail } = props.panel;

    return (
        <div className={classes['panel-thumbnail']}>
            <div className='panel-thumbnail-image' style={{ backgroundImage: `url(${thumbnail})` }} />
            <div className='panel-thumbnail-title'>{title}</div>
            <div className='panel-thumbnail-description'>{description}</div>
        </div>
    );
}

const PanelThumbnailMemo = React.memo(PanelThumbnail);

function DragItem(props: { panel: IPanelMetaInfo }) {
    const mosaicId = useMosaicId();

    const { classes } = useStyle();

    const [, dispatch] = usePanelLayoutStore();

    const [connectDragSource] = useDragPanel(mosaicId, props.panel);

    const handleClickDragItem = () => {
        dispatch(
            addPanelFromOutside({
                path: [],
                position: 'left',
                originPanelConfig: props.panel,
            }),
        );
    };

    return (
        <div ref={connectDragSource} onClick={handleClickDragItem}>
            <Popover title={<PanelThumbnailMemo panel={props.panel} />} trigger='hover' placement='right'>
                <div className={classes['add-panel-item']}>
                    {props.panel.title}
                    <IconIcMove />
                </div>
            </Popover>
        </div>
    );
}

const DragItemMemo = React.memo(DragItem);

function AddPanel() {
    const { allPanel } = usePanelCatalogContext();
    const { t } = useTranslation('addPanel');
    const { classes } = useStyle();

    return (
        <div>
            <MenuDrawerTitle title={t('addPanel')} />
            <div className={classes['add-panel-content']}>
                {allPanel.map((item) => (
                    <DragItemMemo panel={item} key={item.title} />
                ))}
            </div>
        </div>
    );
}

export default React.memo(AddPanel);
