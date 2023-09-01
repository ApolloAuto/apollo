import React from 'react';
import { useTranslation } from 'react-i18next';
import { SubContent, SubHeader, SubItem } from '../../base/PanelHelpContent';

export function PointCloudHelp() {
    const { t } = useTranslation('panels');

    return (
        <>
            <SubHeader>{t('panelHelpDesc')}</SubHeader>
            <SubContent>{t('pointCloudDescription')}</SubContent>
            <SubHeader>{t('panelHelpAbilityDesc')}</SubHeader>
            <SubItem>
                <div>{t('pointCloudAbilityDescOne')}</div>
                <div>{t('pointCloudAbilityDescTwo')}</div>
                <div>{t('pointCloudAbilityDescThree')}</div>
            </SubItem>
        </>
    );
}
