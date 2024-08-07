/* eslint-disable */
import React from 'react';
import Icon  from '@ant-design/icons';
import { IconComponentProps } from '@ant-design/icons/lib/components/Icon';
import DrakSvg from './Drak';
import LightSvg from './Light';
import { useThemeContext } from '@dreamview/dreamview-theme';

export default function IconPark(props: Omit<IconComponentProps, 'name'> & { name: keyof typeof LightSvg | keyof typeof DrakSvg }) {
    const { name: propName, ...others } = props;
    const theme = useThemeContext()?.theme;

    const name = propName;

    let Comp;
    if (theme === 'drak') {
        Comp = DrakSvg || LightSvg;
    } else {
        Comp = LightSvg || DrakSvg;
    }

    return <Icon component={Comp[name] as any} {...others}/>;
}
