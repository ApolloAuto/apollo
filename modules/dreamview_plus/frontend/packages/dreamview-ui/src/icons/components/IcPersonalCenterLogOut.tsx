/* eslint-disable */
import React from 'react';
import Icon  from '@ant-design/icons';
import { IconProps } from '../type';
import { default as Svg } from '../../svgs/ic_personal_center_log_out.svg';
import { default as SvgDrak } from '../../iconDrak/svgs/ic_personal_center_log_out.svg';
import { useThemeContext } from '@dreamview/dreamview-theme';

const IconIcPersonalCenterLogOut = (props: IconProps) => {
    const theme = useThemeContext()?.theme;
    let Comp = Svg;
    if (theme === 'drak') {
        Comp = SvgDrak;
    }
    return <Icon component={Comp as any} {...props}/>;
};
export default IconIcPersonalCenterLogOut;
