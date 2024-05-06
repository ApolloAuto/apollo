import { Meta, StoryObj } from '@storybook/react';
import { Spin } from './index';

// More on how to set up stories at: https://storybook.js.org/docs/react/writing-stories/introduction
const meta: Meta<typeof Spin> = {
    title: 'Dreamview/Spin',
    component: Spin,
    tags: ['autodocs'],
};

// More on writing stories with args: https://storybook.js.org/docs/react/writing-stories/args
export const Primary: StoryObj<typeof Spin> = {
    args: {},
};

export default meta;
