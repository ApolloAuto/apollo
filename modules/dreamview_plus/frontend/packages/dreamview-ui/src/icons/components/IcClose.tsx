/* eslint-disable */
import React from 'react';
import Icon  from '@ant-design/icons';
import { IconProps } from '../type';
import { default as Svg } from '../../svgs/ic_close.svg';

const IconIcClose = (props: IconProps) => <Icon component={Svg as any} {...props}/>;
export default IconIcClose;
