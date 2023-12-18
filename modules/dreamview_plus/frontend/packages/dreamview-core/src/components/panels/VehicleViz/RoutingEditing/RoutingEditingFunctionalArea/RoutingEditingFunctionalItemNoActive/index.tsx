import React from 'react';
import { useTranslation } from 'react-i18next';
import useStyle from './useStyle';

interface FunctionalItemNoActiveProps {
    functionalItemNoActiveText: string;
}

export default function FunctionalItemNoActive(props: FunctionalItemNoActiveProps) {
    const { functionalItemNoActiveText } = props;

    const { classes, cx } = useStyle();

    const { t } = useTranslation('routeEditing');

    return <div className={classes['routing-editing-item-no-active']}>{t(functionalItemNoActiveText)}</div>;
}
