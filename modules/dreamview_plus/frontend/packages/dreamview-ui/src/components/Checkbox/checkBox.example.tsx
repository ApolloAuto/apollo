import React from 'react';
import { CheckboxChangeEvent, CheckboxProps } from 'antd/es/checkbox';
import { CheckboxValueType } from 'antd/es/checkbox/Group';
import { CheckboxGroup } from '.';
import { Checkbox } from './checkBox';

export function Template(props: CheckboxProps) {
    const onChange = (e: CheckboxChangeEvent) => {
        console.log(`checked = ${e.target.checked}`);
    };

    return <Checkbox onChange={onChange}>Template</Checkbox>;
}
