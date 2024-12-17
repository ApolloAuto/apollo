import React, { useCallback } from 'react';
import { usePanelCatalogContext } from '@dreamview/dreamview-core/src/store/PanelCatalogStore';
import {
    useGetCurrentLayout,
    useMosaicId,
    usePanelLayoutStore,
} from '@dreamview/dreamview-core/src/store/PanelLayoutStore';
import { IPanelMetaInfo } from '@dreamview/dreamview-core/src/components/panels/type/Panel';
import useDragPanel from '@dreamview/dreamview-core/src/util/useDragPanel';
import { IconPark, Popover, OperatePopover } from '@dreamview/dreamview-ui';
import { useTranslation } from 'react-i18next';
import { getPanelTypeByPanelId } from '@dreamview/dreamview-core/src/util/layout';
import { resetLayoutByMode, addPanelFromOutside } from '@dreamview/dreamview-core/src/store/PanelLayoutStore/actions';
import { usePickHmiStore } from '@dreamview/dreamview-core/src/store/HmiStore';
import { downloadTextFile } from '@dreamview/dreamview-core/src/util/clipboart';
import CustomScroll from '@dreamview/dreamview-core/src/components/CustomScroll';
import MenuDrawerTitle from '../Common/MenuDrawerTitle';
import IcExport from '../../../assets/svg/ic_export';
import useStyle from './useStyle';

function PanelThumbnail(props: { panel: IPanelMetaInfo }) {
    const { classes } = useStyle();

    const { title, description, thumbnail } = props.panel;

    return (
        <div
            className={classes['panel-thumbnail']}
            onClick={(event) => {
                event.stopPropagation();
            }}
        >
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
    const [hmi] = usePickHmiStore();

    const [connectDragSource] = useDragPanel(mosaicId, props.panel);

    const handleClickDragItem = () => {
        dispatch(
            addPanelFromOutside({
                mode: hmi.currentMode,
                path: [],
                position: 'left',
                originPanelConfig: props.panel,
            }),
        );
    };

    return (
        <div ref={connectDragSource} onClick={handleClickDragItem}>
            <OperatePopover title={<PanelThumbnailMemo panel={props.panel} />} trigger='hover' placement='right'>
                <div className={classes['add-panel-item']}>
                    {props.panel.title}
                    <IconPark name='IcMove' />
                </div>
            </OperatePopover>
        </div>
    );
}

const DragItemMemo = React.memo(DragItem);

/**
 * @description
 * 将一个对象转换为 JSON 格式，并返回该 JSON 对象。
 * 如果传入的值是字符串类型，则直接返回一个包含该字符串内容的 JSON 对象；
 * 否则，遍历传入的对象，将其中的 'direction'、'splitPercentage' 属性复制到结果 JSON 对象中，
 * 如果有 'first' 或 'second' 属性，则递归调用此函数处理这些属性的值，并将处理后的结果作为子 JSON 对象添加到结果 JSON 对象中。
 *
 * @param value {any} 需要转换的对象，可以是任何类型（包括 undefined）。
 * @returns {object|null} 返回一个 JSON 对象，如果传入的值为 undefined，则返回 null。
 */
function toConfJson(value: any) {
    if (!value) {
        return null;
    }
    const isLefaeNode = typeof value === 'string';
    if (isLefaeNode) {
        return {
            type: getPanelTypeByPanelId(value),
        };
    }
    const result: any = {};
    Object.entries(value).forEach(([key, subValue]) => {
        if (key === 'direction' || key === 'splitPercentage') {
            result[key] = subValue;
            return;
        }
        if (key === 'first' || key === 'second') {
            result[key] = toConfJson(subValue);
        }
    });
    return result;
}

function delObjectKeyQuota(str: string) {
    return str
        .replace(/({\s*)(")/g, (match, $1) => $1)
        .replace(/(,\s*)(")/g, (match, $1) => $1)
        .replace(/":/g, ':');
}
/**
 * 返回一个字符串，前缀为"layout:"。
 *
 * @param str 要添加前缀的字符串
 * @returns 返回一个新的字符串，包含"layout:"和原始字符串。
 */
function withPrefix(str: string) {
    return `layout: ${str}`;
}

function AddPanel() {
    const { allPanel } = usePanelCatalogContext();
    const currentLayout = useGetCurrentLayout();
    const [, dispatch] = usePanelLayoutStore();
    const { t } = useTranslation('addPanel');
    const { classes } = useStyle();
    const [hmi] = usePickHmiStore();

    const onResetLayout = useCallback(() => {
        dispatch(resetLayoutByMode({ mode: hmi.currentMode }));
    }, [hmi.currentMode]);

    const download = () => {
        downloadTextFile(
            `${hmi.currentMode} Layout.conf`,
            withPrefix(delObjectKeyQuota(JSON.stringify(toConfJson(currentLayout), null, 4))),
        );
    };

    const extra = [
        <Popover key='resetLayout' trigger='hover' content={t('resetLayout')}>
            <span>
                <IconPark name='Reset' onClick={onResetLayout} />
            </span>
        </Popover>,
        <Popover key='exportLayout' trigger='hover' content={t('exportLayout')}>
            <span>
                <IcExport onClick={download} />
            </span>
        </Popover>,
    ];

    return (
        <>
            <MenuDrawerTitle extra={extra} title={t('addPanel')} />
            <CustomScroll className={classes['add-panel-content']}>
                {allPanel.map((item) => (
                    <DragItemMemo panel={item} key={item.title} />
                ))}
            </CustomScroll>
        </>
    );
}

export default React.memo(AddPanel);
