/* eslint-disable */
import React from 'react';
import Icon  from '@ant-design/icons';
import { IconProps } from '../type';
import { default as Svg } from '../../svgs/upload.svg';

const IconUpload = (props: IconProps) => <Icon component={Svg as any} {...props}/>;
export default IconUpload;
