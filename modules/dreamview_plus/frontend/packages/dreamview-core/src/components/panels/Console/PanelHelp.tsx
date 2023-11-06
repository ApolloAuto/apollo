import React from 'react';
import { useTranslation } from 'react-i18next';

import { useMakeStyle } from '@dreamview/dreamview-theme';

function useStyle() {
    const hoc = useMakeStyle((theme) => ({
        'desc-container': {
            padding: '25px 8px',
        },
        'desc-title': {
            ...theme.tokens.typography.title,
            color: theme.tokens.colors.fontColor1,
            marginBottom: '6px',
        },
        'desc-content': {
            ...theme.tokens.typography.title,
        },
    }));
    return hoc();
}

interface IPanelHelp {
    description: string;
}

function PanelHelp(props: IPanelHelp) {
    const { t } = useTranslation('panels');
    const { classes } = useStyle();
    return (
        <div className={classes['desc-container']}>
            <div className={classes['desc-title']}>{t('descriptionTitle')}</div>
            <div className={classes['desc-content']}>{props.description}</div>
        </div>
    );
}

export default React.memo(PanelHelp);
