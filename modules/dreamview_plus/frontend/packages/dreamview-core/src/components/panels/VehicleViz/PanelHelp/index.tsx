import React from 'react';
import { useTranslation } from 'react-i18next';
import { IconIcAmplification, IconIcReduce, IconIcCoverageHover } from '@dreamview/dreamview-ui';
import { SubContent, SubHeader } from '../../base/PanelHelpContent';
import useStyle from '../useStyle';

export function VehicleVizPanelHelpOrigin() {
    const { t } = useTranslation('panels');
    const { classes } = useStyle();

    return (
        <>
            <SubHeader>{t('panelHelpDesc')}</SubHeader>
            <SubContent>{t('vehicleVizDescription')}</SubContent>
            <SubHeader>{t('panelHelpAbilityDesc')}</SubHeader>
            <div className={classes['panel-desc-item']}>
                <div className={classes['panel-desc-item-left']}>
                    <span className={classes['viz-btn-item']}>
                        <IconIcCoverageHover />
                    </span>
                </div>
                <div className={classes['panel-desc-item-right']}>{t('layerMenuDescription')}</div>
            </div>

            <div className={classes['panel-desc-item']}>
                <div className={classes['panel-desc-item-left']}>
                    <span className={classes['viz-btn-item']}>D</span>
                </div>
                <div className={classes['panel-desc-item-right']}>{t('viewSwitchDescription')}</div>
            </div>

            <div className={classes['panel-desc-item']}>
                <div className={classes['panel-desc-item-left']}>
                    <div className={classes['view-menu-btn-container']}>
                        <IconIcAmplification style={{ fontSize: '16px', color: '#96A5C1', marginBottom: '8px' }} />
                        <IconIcReduce style={{ fontSize: '16px', color: '#96A5C1' }} />
                    </div>
                </div>
                <div className={classes['panel-desc-item-right']}>{t('viewBtnDescription')}</div>
            </div>
        </>
    );
}

export const VehicleVizPanelHelp = React.memo(VehicleVizPanelHelpOrigin);
