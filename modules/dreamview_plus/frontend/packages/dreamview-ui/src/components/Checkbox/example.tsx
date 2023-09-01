import React from 'react';
import { CheckboxChangeEvent, CheckboxGroupProps, CheckboxProps } from 'antd/es/checkbox';
import { CheckboxValueType } from 'antd/es/checkbox/Group';
import { CheckboxGroup } from '.';

export function Template(props: CheckboxGroupProps) {
    const options = [
        { label: 'Apple', value: 'Apple' },
        { label: 'Pear', value: 'Pear' },
        { label: 'Orange', value: 'Orange' },
        { label: 'Banana', value: 'Banana' },
    ];

    const onChange = (checkedValues: CheckboxValueType[]) => {
        console.log('checked = ', checkedValues);
    };

    return <CheckboxGroup options={options} defaultValue={['Pear']} onChange={onChange} />;
}
