import React, { useCallback, useEffect, useMemo, useState } from 'react';
import { usePanelContext } from '@dreamview/dreamview-core/src/components/panels/base/store/PanelStore';
import { Popover } from '@dreamview/dreamview-ui';
import { useTranslation } from 'react-i18next';
import { RoutingEditingBtn, CommonRoutingEditingBtn } from './RoutingEditingBtns';
import useStyle from './useStyle';
import { useNavigate } from '../../../../../util/SimpleRouter';
import { usePickHmiStore, HMIModeOperation } from '../../../../../store/HmiStore';
import RoutingEditingFunctionalFavorite from '../RoutingEditingFunctionalArea/RoutingEditingFunctionalFavorite';
import { CommonRoutingOrigin } from '../RoutingEditingFunctionalArea/types';

export function RoutingEditingOpArea() {
    const { classes, cx } = useStyle();

    const navigate = useNavigate();

    const [hmi] = usePickHmiStore();

    const panelContext = usePanelContext();

    const { enterFullScreen } = panelContext;

    const { t } = useTranslation('routeEditing');

    const popoverText = { popoverDisableText: t('routeEditingBtnOther') };

    const [commonRoutingHovered, setCommonRoutingHovered] = useState(false);

    const [commonRoutingClicked, setCommonRoutingClicked] = useState(false);

    const [routingDisable, setRoutingDisable] = useState(!(hmi.currentMap && hmi.currentVehicle));

    const handleHoverChange = (commonRoutingIsOpen: boolean) => {
        setCommonRoutingHovered(commonRoutingIsOpen);
        setCommonRoutingClicked(false);
    };

    const handleClickChange = (commonRoutingIsOpen: boolean) => {
        setCommonRoutingHovered(false);
        setCommonRoutingClicked(commonRoutingIsOpen);
    };

    const destroyFunFavoriteNotFullScreen = () => {
        setCommonRoutingClicked(false);
    };

    const functionalFavoriteNotFullScreen = (
        <RoutingEditingFunctionalFavorite
            activeOrigin={CommonRoutingOrigin.FROM_NOT_FULLSCREEN}
            destroyFunFavoriteNotFullScreen={destroyFunFavoriteNotFullScreen}
        />
    );

    const editRouting = useCallback(() => {
        enterFullScreen();
        setTimeout(() => {
            navigate('/routing');
        });
    }, [enterFullScreen, navigate]);

    useEffect(() => {
        if (
            hmi.currentOperation === HMIModeOperation.SCENARIO ||
            hmi.currentOperation === HMIModeOperation.SIM_CONTROL
        ) {
            setRoutingDisable(false);
        } else {
            setRoutingDisable(true);
        }
    }, [hmi]);
    return (
        <div className={cx(routingDisable && classes['routing-editing-disable'], classes['routing-editing-op-area'])}>
            {routingDisable ? (
                <>
                    <Popover
                        content={popoverText?.popoverDisableText}
                        trigger='hover'
                        placement='top'
                        rootClassName={classes['routing-editing-btn-popover']}
                    >
                        <>
                            <RoutingEditingBtn onClick={editRouting} />
                        </>
                    </Popover>
                    <Popover
                        content={popoverText?.popoverDisableText}
                        trigger='hover'
                        placement='top'
                        rootClassName={classes['routing-editing-btn-popover']}
                    >
                        <>
                            <CommonRoutingEditingBtn />
                        </>
                    </Popover>
                </>
            ) : (
                <>
                    <RoutingEditingBtn onClick={editRouting} />
                    <Popover
                        content={t('routeCreateCommonBtn')}
                        trigger='hover'
                        placement='top'
                        open={commonRoutingHovered}
                        onOpenChange={handleHoverChange}
                        rootClassName={classes['routing-editing-btn-popover']}
                    >
                        <Popover
                            content={functionalFavoriteNotFullScreen}
                            trigger='click'
                            placement='rightTop'
                            open={commonRoutingClicked}
                            onOpenChange={handleClickChange}
                            destroyTooltipOnHide
                        >
                            <>
                                <CommonRoutingEditingBtn />
                            </>
                        </Popover>
                    </Popover>
                </>
            )}
        </div>
    );
}
