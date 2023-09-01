/* eslint-disable */
import React from 'react';
import Icon  from '@ant-design/icons';
import { IconProps } from '../type';
import { default as Svg } from '../../svgs/overview.svg';

const IconOverview = (props: IconProps) => <Icon component={Svg} {...props}/>;
export default IconOverview;
