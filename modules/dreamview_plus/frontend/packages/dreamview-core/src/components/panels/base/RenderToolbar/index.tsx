import React, { PropsWithChildren, useCallback, useContext, useMemo, useState } from 'react';
import {
    MosaicPath,
    MosaicContext,
    MosaicWindowContext,
    getAndAssertNodeAtPathExists,
    MosaicDirection,
} from 'react-mosaic-component';
import { IPanelMetaInfo } from '@dreamview/dreamview-core/src/components/panels/type/Panel';
import { genereatePanelId } from '@dreamview/dreamview-core/src/util/layout';
import {
    IconIcBottomRailDeleteHover,
    IconIcFullscreen,
    IconIcSplitDown,
    IconIcSplitRight,
    IconIcHelpNormal,
    IconIcSettingNormal,
    IconIcMoveHover,
    IconIcEitFullScreen,
    IconIcDelete,
    Modal,
} from '@dreamview/dreamview-ui';
import { Popover } from '@dreamview/dreamview-ui/src/components/Popover';
import { useTranslation } from 'react-i18next';
import useStyle from './style';
import './index.less';
import { usePanelInfoStore } from '../../../../store/PanelInfoStore';
import { addSelectedPanelId, deleteSelectedPanelId } from '../../../../store/PanelInfoStore/actions';
import { usePanelTileContext } from '../../../../store/PanelInnerStore/PanelTileStore';

type SubscribeInfo = {
    name?: string;
    channel?: string;
    needChannel: boolean;
};

export interface RenderToolbarProps {
    panel?: IPanelMetaInfo;
    path?: MosaicPath;
    panelId?: string;
    inFullScreen?: boolean;
    helpContent?: React.ReactNode;
    updateChannel?: (newChannel: SubscribeInfo) => void;
    name?: string;
    customToolBar?: React.JSX.Element;
}

function RenderToolbar(props: PropsWithChildren<RenderToolbarProps>) {
    const { t } = useTranslation('panels');
    const { classes, cx } = useStyle();
    const { path, panel, panelId, inFullScreen, helpContent, updateChannel, name } = props;
    const {
        mosaicWindowActions: { connectDragSource: mosaicConnectDragSource },
    } = useContext(MosaicWindowContext);
    const { mosaicActions } = useContext(MosaicContext);
    const { onClosePanel, enterFullScreen, exitFullScreen } = usePanelTileContext();
    const [isModalOpen, setIsModalOpen] = useState(false);
    const [panelInfoState, panelInfoDispatch] = usePanelInfoStore();
    const isSelected = useMemo(() => panelInfoState?.selectedPanelIds?.has(panelId) ?? false, [panelInfoState]);

    const connectDragSource = (el: any) => {
        if (path.length > 0) {
            mosaicConnectDragSource(el);
        }
    };

    const showModal = () => {
        setIsModalOpen(true);
    };

    const handleOk = () => {
        setIsModalOpen(false);
    };

    const handleCancel = () => {
        setIsModalOpen(false);
    };

    const split = useCallback(
        (direction: MosaicDirection) => {
            const root = mosaicActions.getRoot();
            const newId = genereatePanelId(panel?.type);
            mosaicActions.replaceWith(path, {
                direction,
                second: newId,
                first: getAndAssertNodeAtPathExists(root, path),
            });
        },
        [path, mosaicActions, panel?.type],
    );

    const onSplitRow = useCallback(() => {
        split('row');
    }, [split]);

    const onSplitColumn = useCallback(() => {
        split('column');
    }, [split]);

    const onRemove = useCallback(() => {
        onClosePanel();
    }, [onClosePanel]);

    const operate = (
        <div className={classes['mosaic-custom-toolbar-operate-popover']}>
            <div onClick={onSplitRow} className={classes['mosaic-custom-toolbar-operate-popover-select']}>
                <IconIcSplitRight />
                {t('operateSplitRight')}
            </div>
            <div onClick={onSplitColumn} className={classes['mosaic-custom-toolbar-operate-popover-select']}>
                <IconIcSplitDown />
                {t('operateSplitDown')}
            </div>
            <div onClick={enterFullScreen} className={classes['mosaic-custom-toolbar-operate-popover-select']}>
                <IconIcFullscreen />
                {t('operateFullScreen')}
            </div>
            <div
                onClick={onRemove}
                className={cx(
                    classes['mosaic-custom-toolbar-operate-popover-select'],
                    classes['mosaic-custom-toolbar-operate-popover-select-remove'],
                )}
            >
                <IconIcDelete />
                {t('operateRemovePanel')}
            </div>
        </div>
    );

    const onToolbarClick: React.MouseEventHandler<HTMLDivElement> = useCallback(
        (event) => {
            if (!isSelected) {
                panelInfoDispatch(addSelectedPanelId(panelId));
            }
        },
        [isSelected, panelInfoDispatch],
    );

    return (
        <div onClick={onToolbarClick} className={classes['mosaic-custom-toolbar-root']} ref={connectDragSource as any}>
            {!inFullScreen ? (
                <div className={classes['mosaic-custom-toolbar-operate']}>
                    <div onClick={showModal} className={classes['mosaic-custom-toolbar-operate-item']}>
                        <IconIcHelpNormal />
                    </div>
                    <div className={classes['mosaic-custom-toolbar-operate-item']}>
                        <Popover
                            trigger='hover'
                            rootClassName={classes['mosaic-custom-toolbar-popover']}
                            content={operate}
                        >
                            <IconIcSettingNormal />
                        </Popover>
                    </div>
                    <div className={classes['mosaic-custom-toolbar-operate-item']}>
                        <Popover
                            trigger='hover'
                            rootClassName={classes['mosaic-custom-toolbar-icmove']}
                            content={t('pressTips')}
                        >
                            <IconIcMoveHover />
                        </Popover>
                    </div>
                </div>
            ) : (
                <div onClick={exitFullScreen} className={classes['mosaic-custom-toolbar-exit-fullscreen']}>
                    <IconIcEitFullScreen />
                    &nbsp;Exit FullScreen
                </div>
            )}

            <div className={classes['mosaic-custom-toolbar-title']}>
                {props.panel?.title}
                &nbsp;
                {props.children}
            </div>
            <Modal
                width={816}
                title={props.panel?.title}
                footer={null}
                open={isModalOpen}
                onOk={handleOk}
                onCancel={handleCancel}
                className='dreamview-modal-panel-help'
            >
                {helpContent ?? (
                    <>
                        <p>Some contents...</p>
                        <p>Some contents...</p>
                        <p>Some contents...</p>
                    </>
                )}
            </Modal>
        </div>
    );
}

export default React.memo(RenderToolbar);
