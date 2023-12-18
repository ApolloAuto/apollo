import { Meta, StoryObj } from '@storybook/react';
import { Switch } from './index';

// More on how to set up stories at: https://storybook.js.org/docs/react/writing-stories/introduction
const meta: Meta<typeof Switch> = {
    title: 'Dreamview/Switch',
    component: Switch,
    tags: ['autodocs'],
};

// More on writing stories with args: https://storybook.js.org/docs/react/writing-stories/args
export const Primary: StoryObj<typeof Switch> = {
    // args: {
    //     primary: true,
    //     label: 'Primary',
    //     backgroundColor: 'blue',
    // },
};

export const Secondary: StoryObj<typeof Switch> = {
    // args: {
    //     label: 'Secondary',
    // },
};

export default meta;
