/* eslint-disable */
import React from 'react';
import Icon  from '@ant-design/icons';
import { IconProps } from '../type';
import { default as Svg } from '../../svgs/reset.svg';
import { default as SvgDrak } from '../../iconDrak/svgs/reset.svg';
import { useThemeContext } from '@dreamview/dreamview-theme';

const IconReset = (props: IconProps) => {
    const theme = useThemeContext()?.theme;
    let Comp = Svg;
    if (theme === 'drak') {
        Comp = SvgDrak;
    }
    return <Icon component={Comp as any} {...props}/>;
};
export default IconReset;
