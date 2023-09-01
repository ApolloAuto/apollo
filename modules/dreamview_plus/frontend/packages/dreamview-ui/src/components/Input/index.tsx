import React from 'react';
import { Input as InternalInput, InputProps } from 'antd';
import { TextAreaProps } from 'antd/es/input';
import { getPrefixCls } from '../../tools/prefixCls/prefixCls';

const { TextArea: InternalTextArea } = InternalInput;

export function Input(props: InputProps) {
    const { prefixCls: customizePrefixCls, children, ...rest } = props;
    const prefixCls = getPrefixCls('input', customizePrefixCls);

    return (
        <InternalInput prefixCls={prefixCls} {...rest}>
            {children}
        </InternalInput>
    );
}
Input.propTypes = {};

Input.defaultProps = {};

Input.displayName = 'Input';

export function TextArea(props: TextAreaProps) {
    const { prefixCls: customizePrefixCls, children, ...rest } = props;
    const prefixCls = getPrefixCls('text-area', customizePrefixCls);

    return (
        <InternalTextArea prefixCls={prefixCls} {...rest}>
            {children}
        </InternalTextArea>
    );
}
TextArea.propTypes = {};

TextArea.defaultProps = {};

TextArea.displayName = 'Text-Area';
