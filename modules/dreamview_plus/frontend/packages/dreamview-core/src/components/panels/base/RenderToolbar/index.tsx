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
import { OperatePopover, IconPark, Modal } from '@dreamview/dreamview-ui';
import { Popover } from '@dreamview/dreamview-ui/src/components/Popover';
import { useTranslation } from 'react-i18next';
import useStyle from './style';
import './index.less';
import { usePanelInfoStore } from '../../../../store/PanelInfoStore';
import { addSelectedPanelId } from '../../../../store/PanelInfoStore/actions';
import { usePanelTileContext } from '../../../../store/PanelInnerStore/PanelTileStore';
import { SubHeader } from '../PanelHelpContent';

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
    const { path, panel, panelId, inFullScreen, helpContent } = props;
    const {
        mosaicWindowActions: { connectDragSource: mosaicConnectDragSource },
    } = useContext(MosaicWindowContext);
    const { mosaicActions } = useContext(MosaicContext);
    const { onClosePanel, enterFullScreen, exitFullScreen } = usePanelTileContext();
    const [isModalOpen, setIsModalOpen] = useState(false);
    const [panelInfoState, panelInfoDispatch] = usePanelInfoStore();
    const isSelected = useMemo(() => panelInfoState?.selectedPanelIds?.has(panelId) ?? false, [panelInfoState]);
    const keyHandlers = useMemo(() => {
        const panelKeyHandlers = panelInfoState.keyHandlerMap.get(panelId) ?? [];
        const gloableKeyHandlers =
            panelInfoState.globalKeyhandlers.size === 0 ? [] : Array.from(panelInfoState.globalKeyhandlers.values());
        return [...gloableKeyHandlers, ...panelKeyHandlers];
    }, [panelInfoState]);

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
                <IconPark name='IcSplitRight' />
                {t('operateSplitRight')}
            </div>
            <div onClick={onSplitColumn} className={classes['mosaic-custom-toolbar-operate-popover-select']}>
                <IconPark name='IcSplitDown' />
                {t('operateSplitDown')}
            </div>
            <div onClick={enterFullScreen} className={classes['mosaic-custom-toolbar-operate-popover-select']}>
                <IconPark name='IcFullscreen' />
                {t('operateFullScreen')}
            </div>
            <div
                onClick={onRemove}
                className={cx(
                    classes['mosaic-custom-toolbar-operate-popover-select'],
                    classes['mosaic-custom-toolbar-operate-popover-select-remove'],
                )}
            >
                <IconPark name='IcDelete' />
                {t('operateRemovePanel')}
            </div>
        </div>
    );

    const onToolbarClick: React.MouseEventHandler<HTMLDivElement> = useCallback(() => {
        if (!isSelected) {
            panelInfoDispatch(addSelectedPanelId(panelId));
        }
    }, [isSelected, panelInfoDispatch]);
    const KeyDescriptor = useMemo(() => {
        const handlersDesc = keyHandlers?.map((handler) => {
            let functionalKey;
            if (handler.functionalKey) {
                functionalKey = <span className={classes['btn-item']}>{handler.functionalKey}</span>;
            }

            let keys;
            if (handler.keys) {
                keys = (
                    <>
                        {handler.keys.map((key, index) => (
                            <span key={key} className={classes['btn-item']}>
                                {key}
                            </span>
                        ))}
                    </>
                );
            }

            return (
                <div className={classes['panel-desc-item']} key={handler.keys.join('')}>
                    <div className={classes['panel-desc-item-left']}>
                        {functionalKey}
                        {keys}
                    </div>
                    <div className={classes['panel-desc-item-right']}>{handler.discriptor}</div>
                </div>
            );
        });

        return (
            <>
                <SubHeader>{t('shortCut')}</SubHeader>
                {handlersDesc}
            </>
        );
    }, [classes, keyHandlers, t]);

    return (
        <div onClick={onToolbarClick} className={classes['mosaic-custom-toolbar-root']} ref={connectDragSource as any}>
            {!inFullScreen ? (
                <div className={classes['mosaic-custom-toolbar-operate']}>
                    <div onClick={showModal} className={classes['mosaic-custom-toolbar-operate-item']}>
                        <IconPark name='IcHelpNormal' />
                    </div>
                    <div className={classes['mosaic-custom-toolbar-operate-item']}>
                        <OperatePopover
                            trigger='hover'
                            rootClassName={classes['mosaic-custom-toolbar-popover']}
                            content={operate}
                        >
                            <IconPark name='IcSettingNormal' />
                        </OperatePopover>
                    </div>
                    <div className={classes['mosaic-custom-toolbar-operate-item']}>
                        <Popover
                            trigger='hover'
                            rootClassName={classes['mosaic-custom-toolbar-icmove']}
                            content={t('pressTips')}
                        >
                            <IconPark name='IcMoveHover' />
                        </Popover>
                    </div>
                </div>
            ) : (
                <div onClick={exitFullScreen} className={classes['mosaic-custom-toolbar-exit-fullscreen']}>
                    <IconPark name='IcEitFullScreen' />
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
                <div style={{ width: '100%', height: '100%' }}>
                    {helpContent}
                    {KeyDescriptor}
                </div>
            </Modal>
        </div>
    );
}

export default React.memo(RenderToolbar);
