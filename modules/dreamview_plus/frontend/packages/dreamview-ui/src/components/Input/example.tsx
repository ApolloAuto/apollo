import React from 'react';
import { InputProps } from 'antd';
import { Input, TextArea } from '.';

export function Template(props: InputProps) {
    return (
        <div>
            <Input placeholder='input' />
            <TextArea placeholder='text-area' />
        </div>
    );
}
