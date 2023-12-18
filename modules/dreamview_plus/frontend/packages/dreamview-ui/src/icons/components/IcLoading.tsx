/* eslint-disable */
import React from 'react';
import Icon  from '@ant-design/icons';
import { IconProps } from '../type';
import { default as Svg } from '../../svgs/ic_loading.svg';

const IconIcLoading = (props: IconProps) => <Icon component={Svg as any} {...props}/>;
export default IconIcLoading;
