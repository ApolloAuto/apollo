import React, { useState } from 'react';
import RcTable, { TableProps } from 'rc-table';
import { useTranslation } from 'react-i18next';
import useStyle from './useStyle';

export default function Table(props: TableProps) {
    const { classes, cx } = useStyle();
    const { t } = useTranslation('table');
    return <RcTable {...props} emptyText={t('empty')} className={cx(classes['profile-table'], props.className)} />;
}
