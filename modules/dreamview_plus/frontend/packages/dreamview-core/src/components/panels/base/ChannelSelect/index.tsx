import { Select } from '@dreamview/dreamview-ui';
import { SelectProps } from 'antd';
import React from 'react';
import { useTranslation } from 'react-i18next';
import './index.less';

function ChannelSelect(props: SelectProps) {
    const { t } = useTranslation('panels');

    return (
        <Select
            className='channel-select'
            style={{
                width: 188,
                height: 28,
                lineHeight: 28,
            }}
            placeholder={t('selectChannel')}
            popupMatchSelectWidth={false}
            dropdownStyle={{
                width: 322,
            }}
            {...props}
        />
    );
}

export default React.memo(ChannelSelect);
