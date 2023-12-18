import React, { useState } from 'react';
import { RadioChangeEvent, RadioGroupProps } from 'antd';
import { Radio } from '.';
import { RadioGroup } from './radioGroup';

export function RadioGroupTemplate(props: RadioGroupProps) {
    const [value, setValue] = useState(1);

    const onChange = (e: RadioChangeEvent) => {
        setValue(e.target.value);
    };
    return (
        <RadioGroup onChange={onChange} value={value}>
            <Radio value={1}>A</Radio>
            <Radio value={2}>B</Radio>
            <Radio value={3}>C</Radio>
            <Radio value={4}>D</Radio>
        </RadioGroup>
    );
}
