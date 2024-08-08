import React from 'react';
import { useTranslation } from 'react-i18next';
import IconPark from '../../../IconPark';

export default function NoDataPlaceHolder() {
    const { t } = useTranslation('panels');

    return (
        <div
            style={{
                height: 68,
                lineHeight: '68px',
                textAlign: 'center',
            }}
        >
            <IconPark
                name='IcWarningMessage'
                style={{
                    color: '#FF8D26',
                    fontSize: 16,
                    marginRight: '8px',
                }}
            />
            <span
                style={{
                    color: '#A6B5CC',
                    textAlign: 'center',
                    fontSize: 14,
                    fontFamily: 'PingFangSC-Regular',
                }}
            >
                {t('noData')}
            </span>
        </div>
    );
}
