import { Meta, StoryObj } from '@storybook/react';
import { Progress } from './index';

// More on how to set up stories at: https://storybook.js.org/docs/react/writing-stories/introduction
const meta: Meta<typeof Progress> = {
    title: 'Dreamview/Progress',
    component: Progress,
    tags: ['autodocs'],
};

// More on writing stories with args: https://storybook.js.org/docs/react/writing-stories/args
export const PositionExample = Progress.bind({});
// PositionExample.args = {};

export default meta;
