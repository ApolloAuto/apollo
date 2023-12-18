import React from 'react';
import { useTranslation } from 'react-i18next';
import { SubContent, SubHeader, SubItem } from '../../base/PanelHelpContent';

export function DashBoardHelpOrigin() {
    const { t } = useTranslation('panels');

    return (
        <>
            <SubHeader>{t('descriptionTitle')}</SubHeader>
            <SubContent>{t('dashBoardDesc')}</SubContent>
            <SubHeader>{t('panelHelpAbilityDesc')}</SubHeader>
            <SubItem>{t('dashBoardDescription')}</SubItem>
        </>
    );
}

export const DashBoardHelp = React.memo(DashBoardHelpOrigin);
