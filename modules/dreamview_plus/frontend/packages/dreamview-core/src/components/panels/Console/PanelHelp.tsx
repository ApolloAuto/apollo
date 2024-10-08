import React from 'react';
import { useTranslation } from 'react-i18next';

import { makeStyles } from '@dreamview/dreamview-theme';

const useStyle = makeStyles((theme) => ({
    'desc-container': {
        padding: '25px 8px',
    },
    'desc-title': {
        ...theme.tokens.typography.title,
        color: theme.tokens.colors.fontColor5,
        marginBottom: '6px',
    },
    'desc-content': {
        ...theme.tokens.typography.title,
    },
}));

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
