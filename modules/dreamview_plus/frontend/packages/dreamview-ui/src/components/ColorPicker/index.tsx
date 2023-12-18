import React from 'react';
import { ColorPicker as InternalColorPicker, ColorPickerProps } from 'antd';
import { getPrefixCls } from '../../tools/prefixCls/prefixCls';

export function ColorPicker(props: ColorPickerProps) {
    const { prefixCls: customizePrefixCls, ...rest } = props;
    const prefixCls = getPrefixCls('color-picker', customizePrefixCls);

    return <InternalColorPicker prefixCls={prefixCls} {...rest} />;
}

ColorPicker.propTypes = {};

ColorPicker.defaultProps = {};

ColorPicker.displayName = 'ColorPicker';
