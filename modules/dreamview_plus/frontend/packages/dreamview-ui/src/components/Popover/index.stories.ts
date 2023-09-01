import { Meta, StoryObj } from '@storybook/react';
import { FunctionComponent } from 'react';
import { PopoverProps } from 'antd';
import { Popover } from './index';
import { Position } from './example';

// More on how to set up stories at: https://storybook.js.org/docs/react/writing-stories/introduction
const meta: Meta<typeof Popover> = {
    title: 'Dreamview/Popover',
    component: Popover as FunctionComponent<PopoverProps>,
    tags: ['autodocs'],
};

// More on writing stories with args: https://storybook.js.org/docs/react/writing-stories/args
export const PositionExample = Position.bind({});
// PositionExample.args = {};

export default meta;
