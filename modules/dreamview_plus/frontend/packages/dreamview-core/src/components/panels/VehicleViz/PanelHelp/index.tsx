import React from 'react';
import { useTranslation } from 'react-i18next';
import { IconIcAmplification, IconIcReduce, IconIcCoverageHover } from '@dreamview/dreamview-ui';
import { SubContent, SubHeader } from '../../base/PanelHelpContent';
import useStyle from '../useStyle';

export function VehicleVizPanelHelpOrigin() {
    const { t } = useTranslation('panels');
    const { classes, cx } = useStyle();

    return (
        <>
            <SubHeader>{t('panelHelpDesc')}</SubHeader>
            <SubContent>{t('vehicleVizDescription')}</SubContent>
            <SubHeader>{t('panelHelpAbilityDesc')}</SubHeader>
            <div className={classes['panel-desc-item']}>
                <div className={classes['panel-desc-item-left']}>
                    <span className={classes['viz-help-btn-item']}>
                        <IconIcCoverageHover />
                    </span>
                </div>
                <div className={classes['panel-desc-item-right']}>{t('layerMenuDescription')}</div>
            </div>

            <div className={classes['panel-desc-item']}>
                <div className={classes['panel-desc-item-left']}>
                    <span className={classes['viz-help-btn-item']}>D</span>
                </div>
                <div className={classes['panel-desc-item-right']}>{t('viewSwitchDescription')}</div>
            </div>

            <div className={classes['panel-desc-item']}>
                <div className={classes['panel-desc-item-left']}>
                    <span className={cx(classes['viz-help-btn-item'], classes['viz-btn-item-flex'])}>
                        <IconIcAmplification style={{ fontSize: '16px', marginBottom: '8px' }} />
                        <IconIcReduce style={{ fontSize: '16px' }} />
                    </span>
                </div>
                <div className={classes['panel-desc-item-right']}>{t('viewBtnDescription')}</div>
            </div>
        </>
    );
}

export const VehicleVizPanelHelp = React.memo(VehicleVizPanelHelpOrigin);
